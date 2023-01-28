#include <CAN.h>
#include <ArduinoRS485.h>
#include "wiring_private.h"

// CAN Object Dictionary
#define INTERP_TIME 0x60C2
#define OP_MODE 0x6060
#define CONTROL_WORD 0x6040
#define STATUS_WORD 0x6041
#define TARGET_POS 0x607A
#define ACTUAL_POS 0x6064
#define CONTROL_PARAMETERS 0x2001
#define POSITION_LIMITS 0x2039

// Object Subindices
#define DRIVE_CONTROL_WORD_0 0x01   // Sub-Index to CONTROL_PARAMETERS, 
#define PRELOAD_POS 0x01            // Sub-Index to POSITION_LIMITS, stores new position value before calling SET_POSITION event

// Predefined Parameter Values
#define SET_POSITION 0x0A           // Zero_Position_Error and Set_Position, used with CONTROL_PARAMETERS:DRIVE_CONTROL_WORD_0
#define CYCLIC_POS 0x08             // Cyclic Positioning Mode, used with OP_MODE:1
#define DRIVE_SHUTDOWN 0x06
#define DRIVE_ENABLE 0x0F

#define TRUE 1
#define FALSE 0
#define STO_IN A1
#define EN_LED A2

#define ENCODER_COUNTS_PER_UNIT 163840.0     //8192 encoder counts x 20:1 gear ratio
#define TELM_INTERVAL 200                    // Send TELM every n milliseconds
#define TPOS_TIMEOUT 12
#define DREN_TIMEOUT 2000
#define RS485_TIMEOUT 100
#define BRAKING_DECEL .0002
#define RECOVERY_ACCEL .0002
#define COAST_LIMIT 10                      // Coast for up to 10 x TPOS_TIMEOUT (160ms)

#define DISABLED 1
#define TRACKING 2
#define COASTING 3
#define BRAKING 4
#define RECOVERY 5
#define STOPPED 6

Uart RS232Serial (&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0);    // Create new UART instance assigning RX to pin 1 and TX to pin 0


byte CAN_id = 1;
unsigned long last_packet_time = 0;
unsigned long last_TELM_time = 0;
unsigned long last_DREN_time = 0;
unsigned long last_RS485_time = 0;
bool RS485_connection = FALSE;
bool RS232_connection = FALSE;
float segment_velocity = 0;
float braking_velocity = 0;
float recovery_velocity = 0;
float target_pos;
float last_pos;
float calculated_pos;           // substitute for target_pos when coasting, braking and recovering
float current_pos;
unsigned long drive_status;
int axis_state;
bool DREN_flag = FALSE;
bool forward_flag;
int coast_count = 0;
float recovery_distance;
int recovery_count = 0;

union uint32_b{
  uint32_t i;
  byte b[4]; 
};

union float_b{
  float f;
  byte b[4]; 
};

union uint16_b{
  uint16_t i;
  byte b[2]; 
};



void setup()
{
  Serial.begin(115200);
  RS232Serial.begin(115200);
  RS485.begin(115200);
  delay(2000);
  RS485.receive();
  
  pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
  pinMode (5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode (EN_LED, OUTPUT);
  digitalWrite(EN_LED, LOW);
  
  Serial.println("Establishing CAN Communications");
  target_pos = 0;

  // start the CAN bus at 1000 kbps
  if (!CAN.begin(1E6)) 
  {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  delay(100);
  read_CAN();
  
  // Place drive in Disabled State
  Serial.println("Switch drive to Disabled State");
  SDO_send(CAN_id,CONTROL_WORD,0,DRIVE_SHUTDOWN);
  delay(50);
  drive_status =  SDO_read(CAN_id,STATUS_WORD,0);
  Serial.print ("Drive Status = ");
  Serial.println(drive_status,HEX);
  delay(50);

  axis_state = DISABLED;
  
  Serial.println("Set initial position to Zero");
  // Write Preload Position, then trigger SET POSITION event
  SDO_send(CAN_id,POSITION_LIMITS,PRELOAD_POS,0);
  SDO_send(CAN_id,CONTROL_PARAMETERS,DRIVE_CONTROL_WORD_0,SET_POSITION);

  // Setup AMC amplifier for Cyclic Synchronous Positioning at 99 millisecond interval
  // Following setup calls may not be required if power-on defaults have been set in DriveWare
  
  Serial.println("Set synchronous positioning interval");
  SDO_send(CAN_id,INTERP_TIME,0x01,99);      // Interpolation Period = 99
  SDO_send(CAN_id,INTERP_TIME,0x02,0xFD);    // Exponent = -3
 
  
  Serial.println("Enable synchronous positioning mode");
  SDO_send(CAN_id,OP_MODE,0,CYCLIC_POS);
  
  // Read current position from drive
  current_pos = (float) SDO_read(CAN_id,ACTUAL_POS,0) / (float)ENCODER_COUNTS_PER_UNIT;
  Serial.print ("Position read from drive = ");
  Serial.println(current_pos,4);

  target_pos = current_pos;       // Make equal so we don't jump
  last_pos = target_pos;
}

// **********************  Begin main processing loop  ********************************************** 

void loop() 
{
  // Check for and process new serial data packet
  if(serial_packet_read(&target_pos))            // Returns true if there is a packet with new target position
  {
    last_packet_time = millis();
    last_DREN_time = millis();
    coast_count = 0;
 
    if (axis_state == TRACKING)
    {
      SDO_send(CAN_id,TARGET_POS,0,target_pos * ENCODER_COUNTS_PER_UNIT);
      segment_velocity = target_pos - last_pos;
      last_pos = target_pos;
      if (segment_velocity > 0) forward_flag = TRUE; else forward_flag = FALSE;
    }

    if (axis_state == COASTING)
    {
      axis_state = RECOVERY;
      Serial.println ("Switching to RECOVERY state");
      recovery_velocity = segment_velocity;
    }

    if (axis_state == RECOVERY)
    {
      recovery_distance = target_pos - calculated_pos;
      Serial.print("Recovery Distance = ");
      Serial.println(recovery_distance,4);
      Serial.print("Recovery Velocity = ");
      Serial.println (recovery_velocity,4);
      
      // Must add recovery code here
      if(1)     // when Recovery Complete
      { 
        axis_state = TRACKING;
        Serial.println ("TRACKING STATE");
      }
    }
  }

  // Check for TPOS timeout
  if ((millis() - last_packet_time) >= TPOS_TIMEOUT)     // Handle missed packets after TPOS_TIMEOUT
  {
    last_packet_time = millis();        // reset timer to properly detect next TPOS_timeout
    
    if (axis_state == TRACKING)
    {
      axis_state = COASTING;
      Serial.println ("Switching to COASTING state");
      calculated_pos = target_pos;
    }
    if (axis_state == COASTING)
    { 
      calculated_pos += segment_velocity * TPOS_TIMEOUT/8.333;
      SDO_send(CAN_id,TARGET_POS,0,calculated_pos * ENCODER_COUNTS_PER_UNIT);

      // Enter BRAKING state after COAST_LIMIT packets missed
      if (coast_count >= COAST_LIMIT)
      {
        axis_state = BRAKING;             
        Serial.println ("Switching to BRAKING state");
        digitalWrite(EN_LED, LOW);
        braking_velocity = segment_velocity;
      }
      else
      {
        coast_count++;
        Serial.print ("Dropped Packet = ");
        Serial.println (coast_count);  
      }
    }
    if (axis_state == BRAKING)
    {
      if (forward_flag == TRUE)
      {
        // Decelerate axis to a stop
        if (braking_velocity > 0)
          braking_velocity -= BRAKING_DECEL;
        if (braking_velocity < 0)
        {
          braking_velocity = 0;
          axis_state = STOPPED;               // Requires a new TRKi to reset this
          Serial.println ("Switching to STOPPED state");
          digitalWrite(EN_LED, LOW);
        }
      }
      else
      {
        //Decelerate axis to a stop
        if (braking_velocity < 0)
          braking_velocity += BRAKING_DECEL;
        if (braking_velocity > 0)
        {
          braking_velocity = 0;
          axis_state = STOPPED;               // Requires a new TRKi to reset this
          Serial.println ("Switching to STOPPED state");
        }
      }
      calculated_pos += braking_velocity * TPOS_TIMEOUT/8.333;
      SDO_send(CAN_id,TARGET_POS,0,calculated_pos * ENCODER_COUNTS_PER_UNIT);
    }
  }
  
  // Check for DREN timeout
  if ((millis() - last_DREN_time) >= DREN_TIMEOUT)      // Disable drive after DREN_timeout
  {
    if (DREN_flag == TRUE)
    {
      // Place drive in Disabled State
      DREN_flag = FALSE;
      Serial.println("DREN TIMEOUT");
      axis_state = DISABLED;
      Serial.println ("Switching to disabled state");
      digitalWrite(EN_LED, LOW);
      RS485_connection = FALSE;
      RS232_connection = FALSE;
   
      SDO_send(CAN_id,CONTROL_WORD,0,DRIVE_SHUTDOWN);
      delay(50);
      drive_status =  SDO_read(CAN_id,STATUS_WORD,0);
      Serial.print ("Drive Status = ");
      Serial.println(drive_status,HEX);
      delay(50);
    }
  }

  // Send TELM packets at TELM_INTERVAL
  if (millis() - last_TELM_time >= TELM_INTERVAL)       // Read actual position from drive and send TELM
  {
    last_TELM_time = millis();
    current_pos = ((float)SDO_read(CAN_id,ACTUAL_POS,0) / (float)ENCODER_COUNTS_PER_UNIT);
    send_TELM(current_pos);
    
    //Serial.print("TELM Position = ");
    //Serial.println(current_pos,4);
   
  }
  
  read_CAN();       // Clear the CAN buffer and process incoming packets
    
}

//  ***********************  End of main processing loop  ************************************


void SDO_send (byte CAN_id,int SDO_index,byte sub_index,long SDO_data)
{
  union uint32_b data;
  union uint16_b index;
  data.i = SDO_data;
  index.i = SDO_index;
  
  CAN.beginPacket(0x600 + CAN_id);
  CAN.write(0x22);
  CAN.write(index.b[0]);
  CAN.write(index.b[1]);
  CAN.write(sub_index);
  CAN.write(data.b[0]);
  CAN.write(data.b[1]);
  CAN.write(data.b[2]);
  CAN.write(data.b[3]);
  CAN.endPacket();
}

long SDO_read(int CAN_id, int SDO_index, byte sub_index)
{
  union uint32_b data_in;
  union uint16_b index;
  union uint16_b index_in;
  byte sub_index_in;
  int CANpacketSize;
  index.i = SDO_index;
  
  delay(1);       // Allow time for responses to previous CAN traffic 
  read_CAN();     // Clear CAN buffer and process received data before sending read request
  
  // Send SDO Read request
  CAN.beginPacket(0x600 + CAN_id);
  CAN.write(0x40);
  CAN.write(index.b[0]);
  CAN.write(index.b[1]);
  CAN.write(sub_index);
  CAN.write(0x00);
  CAN.write(0x00);
  CAN.write(0x00);
  CAN.write(0x000);
  CAN.endPacket();

  // Wait for packet in response
  while (!CAN.parsePacket());           // This should have a timeout
  // Check that data packet is consistent with SDO_read response (0x42)
  if ((CAN.available() == 8)&&(CAN.read() == 0x42))
  {
    index_in.b[0] = CAN.read();
    index_in.b[1] = CAN.read();
    sub_index_in = CAN.read();
    data_in.b[0]=CAN.read();
    data_in.b[1]=CAN.read();
    data_in.b[2]=CAN.read();
    data_in.b[3]=CAN.read();   
      
    if ((index_in.i == SDO_index)&&(sub_index_in == sub_index))
      return data_in.i;
  }
}
void read_CAN()     //Read CAN receive buffer and repeat until no new data
{
  while (CAN.parsePacket())
  {
    // This could be fleshed out to handle unexpected conditions, errors etc.
    // For now we do this:
    if (CAN.peek() != 0x60) // Check first byte and handle any unexpected return packets
    {
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.print(CAN.read(), HEX);
      Serial.print(" ");
      Serial.println(CAN.read(), HEX);   
    }
  }
}

int serial_packet_read(float *position)
{
  union float_b pos_in;
  union uint16_b checksum;
  char control_byte;
  // read the incoming byte:
  while (RS485.available()>=11)
  {
    if (RS485.read() != 'T') break;
    if (RS485.read() != 'P') break;
    if (RS485.read() != 'O') break;
    if (RS485.read() != 'S') break;
   
    if (RS485_connection != TRUE)
    {
      Serial.println("RS485 Connection Established");
      RS485_connection = TRUE;
    }
    
    control_byte = RS485.read();
    if (control_byte == 'P')
    {
        pos_in.b[0]=RS485.read();
        pos_in.b[1]=RS485.read();
        pos_in.b[2]=RS485.read();
        pos_in.b[3]=RS485.read();
        checksum.b[0] = RS485.read();
        checksum.b[1] = RS485.read();
        if ((pos_in.b[0]+pos_in.b[1]+pos_in.b[2]+pos_in.b[3])==checksum.i)
        {
          *position = pos_in.f;
          return 1;
        }
        else
        {
          //Serial.println("TPOS Checksum Error on RS485");
          return 0;
        }
    }
    else if (control_byte == 'S')
    {
        pos_in.b[0]=RS485.read();
        pos_in.b[1]=RS485.read();
        pos_in.b[2]=RS485.read();
        pos_in.b[3]=RS485.read();
        checksum.b[0] = RS485.read();
        checksum.b[1] = RS485.read();
        if ((pos_in.b[0]+pos_in.b[1]+pos_in.b[2]+pos_in.b[3])==checksum.i)
        {
          SDO_send(1,POSITION_LIMITS,PRELOAD_POS,pos_in.f*ENCODER_COUNTS_PER_UNIT);
          delay(1);
          read_CAN();
          SDO_send(1,CONTROL_PARAMETERS,DRIVE_CONTROL_WORD_0,SET_POSITION);
          delay(1);
          read_CAN();
          Serial.print("Setp = ");
          Serial.println(pos_in.f);
          current_pos = pos_in.f;
          target_pos = last_pos = current_pos;
          segment_velocity = 0;
          return 0;
        }
        else
        {
          //Serial.println("SETP Checksum error on RS485");
          return 0;
        }
    }
    else if (control_byte == 'I')
    {
      checksum.b[0] = RS485.read();
      checksum.b[1] = RS485.read();
      if ((axis_state == DISABLED) || (axis_state == STOPPED))
      {
        axis_state = TRACKING;
        Serial.println("Switching to TRACKING state");
        DREN_flag = TRUE;
        Serial.println("DREN TRUE");
        digitalWrite(EN_LED, HIGH);
        target_pos = current_pos;
        SDO_send(CAN_id,TARGET_POS,0,target_pos * ENCODER_COUNTS_PER_UNIT);
        SDO_send(CAN_id,CONTROL_WORD,0,DRIVE_ENABLE);     // Enable Servo
        delay(50);
        drive_status =  SDO_read(CAN_id,STATUS_WORD,0);
        Serial.print ("Drive Status = ");
        Serial.println(drive_status,HEX);
        coast_count = 0;
        last_DREN_time = millis();
        last_packet_time = millis();   
      }
    }
  }

  while (RS232Serial.available()>=11)
  {
    if (RS232Serial.read() != 'T') break;
    if (RS232Serial.read() != 'P') break;
    if (RS232Serial.read() != 'O') break;
    if (RS232Serial.read() != 'S') break;
    control_byte = RS232Serial.read();
    if (RS232_connection != TRUE)
    {
      Serial.println("RS232 Connection Established");
      RS232_connection = TRUE;
    }
    if (control_byte == 'P')
    {
        pos_in.b[0]=RS232Serial.read();
        pos_in.b[1]=RS232Serial.read();
        pos_in.b[2]=RS232Serial.read();
        pos_in.b[3]=RS232Serial.read();
        checksum.b[0] = RS232Serial.read();
        checksum.b[1] = RS232Serial.read();
        if ((pos_in.b[0]+pos_in.b[1]+pos_in.b[2]+pos_in.b[3])==checksum.i)
        {
          if (RS485_connection != TRUE)    // Ignore received RS232 data when RS 485 data is present
          {
            *position = pos_in.f;
            return 1;
          }
          else
            return 0;
        }
        else
        {
          //Serial.println("TPOS Checksum Error on RS232");
          return 0;
        }
    }
    else if (control_byte == 'S')
    {
        pos_in.b[0]=RS232Serial.read();
        pos_in.b[1]=RS232Serial.read();
        pos_in.b[2]=RS232Serial.read();
        pos_in.b[3]=RS232Serial.read();
        checksum.b[0] = RS232Serial.read();
        checksum.b[1] = RS232Serial.read();
        if ((pos_in.b[0]+pos_in.b[1]+pos_in.b[2]+pos_in.b[3])==checksum.i)
        {
          SDO_send(1,POSITION_LIMITS,PRELOAD_POS,pos_in.f*ENCODER_COUNTS_PER_UNIT);
          SDO_send(1,CONTROL_PARAMETERS,DRIVE_CONTROL_WORD_0,SET_POSITION);
          delay(1);
          read_CAN();
          Serial.print("Setp = ");
          Serial.println(pos_in.f);
          current_pos = pos_in.f;
          target_pos = last_pos = current_pos;
          segment_velocity = 0;
          return 0;
        }
        else
        {
          //Serial.println("SETP Checksum error on RS232");
          return 0;
        }
    }
    else if (control_byte == 'I')
    {
      checksum.b[0] = RS232Serial.read();
      checksum.b[1] = RS232Serial.read();
      if ((axis_state == DISABLED) || (axis_state == STOPPED))
      {
        axis_state = TRACKING;
        Serial.println("Switching to TRACKING state");
        DREN_flag = TRUE;
        Serial.println ("DREN TRUE");
        digitalWrite(EN_LED, HIGH);
        target_pos = current_pos;
        SDO_send(CAN_id,TARGET_POS,0,target_pos * ENCODER_COUNTS_PER_UNIT);
        SDO_send(CAN_id,CONTROL_WORD,0,DRIVE_ENABLE);     // Enable Servo
        delay(50);
        drive_status =  SDO_read(CAN_id,STATUS_WORD,0);
        Serial.print ("Drive Status = ");
        Serial.println(drive_status,HEX);
        coast_count = 0;
        last_DREN_time = millis();
        last_packet_time = millis();    
      }
    }
  }
  return 0;
}

int send_TELM(float pos)
{
    union float_b telm_pos;
    union uint16_b checksum;
    telm_pos.f = pos;
    checksum.i = telm_pos.b[0] + telm_pos.b[1] + telm_pos.b[2] + telm_pos.b[3]; 
    RS485.beginTransmission();
    RS485.write('T');
    RS485.write('E');
    RS485.write('L');
    RS485.write('M');
    RS485.write(telm_pos.b[0]);
    RS485.write(telm_pos.b[1]);
    RS485.write(telm_pos.b[2]);
    RS485.write(telm_pos.b[3]);
    RS485.write(checksum.b[0]);
    RS485.write(checksum.b[1]);
    RS485.endTransmission();

    RS232Serial.write('T');
    RS232Serial.write('E');
    RS232Serial.write('L');
    RS232Serial.write('M');
    RS232Serial.write(telm_pos.b[0]);
    RS232Serial.write(telm_pos.b[1]);
    RS232Serial.write(telm_pos.b[2]);
    RS232Serial.write(telm_pos.b[3]);
    RS232Serial.write(checksum.b[0]);
    RS232Serial.write(checksum.b[1]);
}

void SERCOM3_Handler()
{
  RS232Serial.IrqHandler();
}
