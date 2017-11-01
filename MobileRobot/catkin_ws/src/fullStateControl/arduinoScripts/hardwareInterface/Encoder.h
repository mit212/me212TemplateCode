
// Slave Select pins for encoders 1,2 and 3
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 10;
const int slaveSelectEnc2 = 9;
//const int slaveSelectEnc3 = 9;

void initEncoders() {
  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);
//  pinMode(slaveSelectEnc3, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signals
  digitalWrite(slaveSelectEnc1,HIGH);
  digitalWrite(slaveSelectEnc2,HIGH);
//  digitalWrite(slaveSelectEnc3,HIGH);
  
  SPI.begin();

  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1,HIGH);       // Terminate SPI conversation 
}


void clearEncoderCount() {
  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation   
}

long readEncoder(int encoder) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value; 
   
  noInterrupts();           // disable all interrupts
  
  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1,LOW);      // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);           
    count_3 = SPI.transfer(0x00);           
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    SPI.endTransaction();
    digitalWrite(slaveSelectEnc1,HIGH);     // Terminate SPI conversation 
  }
  
  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  interrupts();             // enable all interrupts
  
  return count_value;
}

uint16_t readAS5147P(int encoder){
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    unsigned int AS5147P_angle;
    noInterrupts();           // disable all interrupts
//    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    // Read encoder 2
    if (encoder == 2) {
      digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
      delayMicroseconds(1);
      SPI.transfer16(0xFFFF);                     // Request count
      digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation     
      
      digitalWrite(slaveSelectEnc2,LOW);      // Begin SPI conversation
      delayMicroseconds(1);
      AS5147P_angle = SPI.transfer16(0xC000);           // Read highest order byte
      digitalWrite(slaveSelectEnc2,HIGH);     // Terminate SPI conversation
      SPI.endTransaction();
      AS5147P_angle = (AS5147P_angle & (0x3FFF));
    }
    uint16_t AS5147P = ( (unsigned long) AS5147P_angle);
    interrupts();             // enable all interrupts
    return AS5147P;
}

