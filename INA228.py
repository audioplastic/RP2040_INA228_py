#############################################################
#
#   Driver to INA228 by @megalloid
#
#   Need to fix: 
#   1) Value of VSHUNT is needed to multiply to 10 that get correct value
#
##############################################################
# from smbus2 import SMBus

# INA228 Address
INA228_PORT = 0
INA228_ADDRESS = 0x00
INA228_SHUNT_OHMS = 0.010
INA228_SHUNT_TEMPCO_VALUE = 0

#############################################################
# INA228 Reset bytes for CONFIG [15] / [14]
# Decription: 
# RST - Generates a system reset that is the same as power-on reset.
# RSTACC - Resets the contents of accumulation registers ENERGY and CHARGE to 0
#############################################################
INA228_RST_NBIT = 15
INA228_RSTACC_NBIT = 14

#############################################################
# INA228 Initial conversion delay CONFIG [13:6]
# Description: Delay for initial ADC conversion in steps of 2 ms
# Values:
# 0x00 - 0 seconds (default)
# 0x01 - 2 ms
# 0xFF - 510ms
#############################################################
INA228_CONVERSION_DELAY = 0x00
INA228_CONVDLY_NBIT = 6

#############################################################
# INA228 Temperature compensation CONFIG [5]
# Description: Enables temperature compensation of an external shunt
# Values:
# 0x00 - disabled (default)
# 0x01 - enabled
#############################################################
INA228_TEMP_COMP = 0x00
INA228_TEMPCOMP_NBIT = 5

#############################################################
# INA228 ADC Shunt full scale range selection CONFIG [4]
# Description: Shunt full scale range selection across IN+ and IN–.
# Values:
# 0x00 - ±163.84 mV (default)
# 0x01 - ±40.96 mV
#############################################################
INA228_ADCRANGE = 0x00
INA228_ADCRANGE_NBIT = 4

#############################################################
# INA228 ADC Mode
# Description: The user can set the MODE bits for continuous or 
# triggered mode on bus voltage, shunt voltage or temperature measurement.
#
# Values:
# 0x00h = Shutdown
# 0x01h = Triggered bus voltage, single shot
# 0x02h = Triggered shunt voltage, single shot
# 0x03h = Triggered shunt voltage and bus voltage, single shot
# 0x04h = Triggered temperature, single shot
# 0x05h = Triggered temperature and bus voltage, single shot
# 0x06h = Triggered temperature and shunt voltage, single shot
# 0x07h = Triggered bus voltage, shunt voltage and temperature, single shot
# 0x08h = Shutdown
# 0x09h = Continuous bus voltage only
# 0x0Ah = Continuous shunt voltage only
# 0x0Bh = Continuous shunt and bus voltage
# 0x0Ch = Continuous temperature only
# 0x0Dh = Continuous bus voltage and temperature
# 0x0Eh = Continuous temperature and shunt voltage
# 0x0Fh = Continuous bus voltage, shunt voltage and temperature (default)
#############################################################
INA228_ADC_MODE = 0x0F
INA228_ADC_MODE_NBIT = 12

#############################################################
# INA228 ADC conversion time of bus voltage meas.
# Description: Sets the conversion time of the bus voltage measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#############################################################
INA228_VBUS_CONV_TIME = 0x05
INA228_VBUS_CONV_TIME_NBIT = 9

#############################################################
# INA228 ADC conversion time of shunt voltage meas.
# Description: Sets the conversion time of the shunt voltage measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#############################################################
INA228_VSHCT_CONV_TIME = 0x05
INA228_VSHCT_CONV_TIME_NBIT = 6

#############################################################
# INA228 ADC conversion time of temperature meas.
# Description: Sets the conversion time of the temperature measurement
#
# Values:
# 0x00h = 50 μs
# 0x01h = 84 μs
# 0x02h = 150 μs
# 0x03h = 280 μs
# 0x04h = 540 μs
# 0x05h = 1052 μs
# 0x06h = 2074 μs
# 0x07h = 4120 μs
#############################################################
INA228_VTCT_CONV_TIME = 0x05
INA228_VTCT_CONV_TIME_NBIT = 3

#############################################################
# INA228 ADC sample averaging count. 
#
# Values: 
# 0x00h = 1
# 0x01h = 4
# 0x02h = 16
# 0x03h = 64
# 0x04h = 128
# 0x05h = 256
# 0x06h = 512
# 0x07h = 1024
#############################################################
INA228_ADC_AVG = 0x00
INA228_AVG_NBIT = 0

#############################################################
# INA228 ADC alerts signal
#############################################################
INA228_ALERT_MEMSTAT = 0
INA228_ALERT_CNVRF = 1
INA228_ALERT_POL = 2
INA228_ALERT_BUSUL = 3
INA228_ALERT_BUSOL = 4
INA228_ALERT_SHNTUL = 5
INA228_ALERT_SHNTOL = 6
INA228_ALERT_TMPOL = 7
INA228_ALERT_MATHOF = 9
INA228_ALERT_CHARGEOF = 10
INA228_ALERT_ENERGYOF = 11
INA228_ALERT_APOL = 12
INA228_ALERT_SLOWALERT = 13
INA228_ALERT_CNVR = 14
INA228_ALERT_ALATCH = 15

class INA228:
    __INA228_CONFIG         = 0x00
    __INA228_ADC_CONFIG     = 0x01
    __INA228_SHUNT_CAL      = 0x02
    __INA228_SHUNT_TEMPCO   = 0x03
    __INA228_VSHUNT         = 0x04
    __INA228_VBUS           = 0x05
    __INA228_DIETEMP        = 0x06
    __INA228_CURRENT        = 0x07
    __INA228_POWER          = 0x08
    __INA228_ENERGY         = 0x09
    __INA228_CHARGE         = 0x0A
    __INA228_DIAG_ALRT      = 0x0B
    __INA228_SOVL           = 0x0C
    __INA228_SUVL           = 0x0D
    __INA228_BOVL           = 0x0E
    __INA228_BUVL               = 0x0F
    __INA228_TEMP_LIMIT         = 0x10
    __INA228_PWR_LIMIT          = 0x11
    __INA228_MANUFACTURER_ID    = 0x3E
    __INA228_DEVICE_ID          = 0x3F

    def __init__(self, i2c_obj, address = INA228_ADDRESS, shunt_ohms = INA228_SHUNT_OHMS):
        self._address = address
        self._i2c = i2c_obj
        self._shunt_ohms = shunt_ohms
    
    def __convert2comp2float(self, twocompdata, nrofbit, factor):
        isnegative = 1
        isnegative = (isnegative << (nrofbit - 1))

        if(twocompdata > isnegative):
            twocompdata = (twocompdata - (2*isnegative)) * factor
        else:
            twocompdata = (twocompdata * factor)

        return twocompdata

    def __to_bytes(self, register_value):
        return [(register_value >> 8) & 0xFF, register_value & 0xFF]

    def read_register40(self, register):
        result = self._i2c.readfrom_mem(self._address, register, 5,)
        register_value = ((result[0] << 32) & 0xFF00000000) | ((result[1] << 24) & 0xFF000000) | ((result[2] << 16) & 0xFF0000) | ((result[3] << 8) & 0xFF00) | (result[4] & 0xFF)
        return register_value

    def read_register24(self, register):
        result = self._i2c.readfrom_mem(self._address, register, 3)
        register_value = ((result[0] << 16) & 0xFF0000) | ((result[1] << 8) & 0xFF00) | (result[2] & 0xFF)
        return register_value

    def read_register16(self, register):
        result = self._i2c.readfrom_mem(self._address, register, 2)
        register_value = ((result[0] << 8) & 0xFF00) | (result[1] & 0xFF)
        return register_value

    def write_register16(self, register, register_value):
        register_bytes = self.__to_bytes(register_value)
        print("write reg 0x%02X  --  val 0x%.4X" % (register, register_value) )
        result = self._i2c.writeto_mem(self._address, register, bytearray(register_bytes) )

    def get_current_lsb(self):
        if(INA228_ADCRANGE == 0):
            temp = 163.84e-3
        else:
            temp = 40.96e-3
        current_lsb = (temp / self._shunt_ohms) / 524288
        return current_lsb

    def get_shunt_conv_factor(self):
        if(INA228_ADCRANGE == 0):
            shunt_conv_factor = 1.25e-6
        else:
            shunt_conv_factor = 5.0e-6
        return shunt_conv_factor

    def reset_all(self):
        config = 1 << INA228_RST_NBIT
        self.write_register16(self.__INA228_CONFIG, config)

    def reset_energy(self):
        config = 1 << INA228_RSTACC_NBIT
        self.write_register16(self.__INA228_CONFIG, config)

    def set_config(self):
        config = (INA228_CONVERSION_DELAY << INA228_CONVDLY_NBIT) | (INA228_TEMP_COMP << INA228_TEMPCOMP_NBIT) | (INA228_ADCRANGE << INA228_ADCRANGE_NBIT)
        self.write_register16(self.__INA228_CONFIG, config)

    def set_adc_config(self):
        config =  (INA228_ADC_MODE << INA228_ADC_MODE_NBIT) | (INA228_VBUS_CONV_TIME << INA228_VBUS_CONV_TIME_NBIT ) | (INA228_VSHCT_CONV_TIME << INA228_VSHCT_CONV_TIME_NBIT) | (INA228_VTCT_CONV_TIME << INA228_VTCT_CONV_TIME_NBIT) | (INA228_ADC_AVG << INA228_AVG_NBIT)
        self.write_register16(self.__INA228_ADC_CONFIG, config)

    def shunt_calib(self):
        calib_value = int(13107.2e6 * self.get_current_lsb() * self._shunt_ohms)
        self.write_register16(self.__INA228_SHUNT_CAL, calib_value)
 
    def shunt_tempco(self):
        self.write_register16(self.__INA228_SHUNT_TEMPCO, INA228_SHUNT_TEMPCO_VALUE)
 
    def configure(self):
        self.set_config()
        time.sleep(0.01)

        self.set_adc_config()
        time.sleep(0.01)

        self.shunt_calib()
        time.sleep(0.01)

        self.shunt_tempco()
        time.sleep(0.01)

        self.reset_energy()
        time.sleep(0.01)
 
    def get_shunt_voltage(self):
        if(INA228_ADCRANGE == 1):
            conversion_factor = 312.5e-9                # nV/LSB
        else:
            conversion_factor = 78.125e-9               # nV/LSB  
        raw = self.read_register24(self.__INA228_VSHUNT)
        return (self.__convert2comp2float(raw >> 4, 20, conversion_factor)) * 10   # Find and fix *10
        
    def print_shunt_voltage(self): 
        print('Shunt drop: ', self.get_shunt_voltage())

    def get_vbus(self):
        conversion_factor = 195.3125e-6                 # uV/LSB
        raw = self.read_register24(self.__INA228_VBUS)
        return self.__convert2comp2float(raw >> 4, 20, conversion_factor)
     
    def print_vbus(self):
        print('VBUS: ', self.get_vbus())

    def get_temp(self):
        conversion_factor = 7.8125e-3
        raw = self.read_register16(self.__INA228_DIETEMP)
        return self.__convert2comp2float(raw, 16, conversion_factor)

    def print_temp(self):
        print('Die temp: ', self.get_temp())

    def get_current(self):        
        raw = self.read_register24(self.__INA228_CURRENT)
        return self.__convert2comp2float(raw >> 4, 20, self.get_current_lsb())

    def print_current(self):
        print('Current: ', self.get_current())

    def get_power(self):
        current_lsb = self.get_current_lsb() 
        raw = self.read_register24(self.__INA228_POWER)
        return 3.2 * raw * current_lsb   

    def print_power(self):   
        print('Power: ', self.get_power())

    def get_energy(self):
        raw = self.read_register40(self.__INA228_ENERGY)
        return raw * 3.2 * 16 * self.get_current_lsb()

    def print_energy(self):
        print('Energy: ', self.get_energy())

    def get_charge(self):
        raw = self.read_register40(self.__INA228_CHARGE)
        return self.__convert2comp2float(raw , 40, 1)

    def print_charge(self):
        print('Charge: ', self.get_charge(), ' Coulombs')

    def get_diag_alerts(self, alert):
        raw = self.read_register16(self.__INA228_DIAG_ALRT)

        if(alert == INA228_ALERT_ALATCH):
            if (raw & 0x1) == 0x0:
                print('MEMSTAT: Checksum error is detected in the device trim memory space')
                return 1

        elif(alert == INA228_ALERT_CNVRF):
            if (raw & 0x2) == 0x1:
                print('CNVRF: Conversion is completed')

        elif(alert == INA228_ALERT_BUSUL):
            if (raw & 0x4) == 0x1:
                print('BUSUL: Bus voltage measurement falls below the threshold limit in the bus under-limit register')

        elif(alert == INA228_ALERT_BUSOL):
            if (raw & 0x8) == 0x1:
                print('BUSOL: Bus voltage measurement exceeds the threshold limit in the bus over-limit register')

        elif(alert == INA228_ALERT_SHNTUL):
            if (raw & 0x10) == 0x1:
                print('SHNTUL: Shunt voltage measurement falls below the threshold limit in the shunt under-limit register')

        elif(alert == INA228_ALERT_SHNTOL):
            if (raw & 0x40) == 0x1:
                print('SHNTOL: Shunt voltage measurement exceeds the threshold limit in the shunt over-limit register')

        elif(alert == INA228_ALERT_TMPOL):
            if (raw & 0x80) == 0x1:
                print('TMPOL: Temperature measurement exceeds the threshold limit in the temperature over-limit register')

        elif(alert == INA228_ALERT_MATHOF):
            if (raw & 0x100) == 0x1:
                print('MATHOF: Arithmetic operation resulted in an overflow error')

        elif(alert == INA228_ALERT_CHARGEOF):
            if (raw & 0x200) == 0x1:
                print('CHARGEOF: 40 bit CHARGE register has overflowed')

        elif(alert == INA228_ALERT_ENERGYOF):
            if (raw & 0x400) == 0x1:
                print('ENERGYOF: 40 bit ENERGY register has overflowed')

        elif(alert == INA228_ALERT_APOL):
            if (raw & 0x800) == 0x1:
                print('APOL: Alert pin polarity inverted (active-high, open-drain)')
            else:
                print('APOL: Alert pin polarity normale (active-low, open-drain)')

        elif(alert == INA228_ALERT_SLOWALERT):
            if (raw & 0x2000) == 0x1:
                print('SLOWALERT: ALERT function is asserted on the completed averaged value. ALERT comparison on averaged value')
            else:
                print('SLOWALERT: ALERT comparison on non-averaged (ADC) value')

        elif(alert == INA228_ALERT_CNVR):
            if (raw & 0x4000) == 0x1:
                print('CNVR: Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed. Enables conversion ready flag on ALERT pin')
            else:
                print('CNVR: Disable conversion ready flag on ALERT pin')

        elif(alert == INA228_ALERT_ALATCH):
            if (raw & 0x8000) == 0x1:
                print('ALATCH: Latched')
            else:
                print('ALATCH: Transparent')

    def set_shunt_overvoltage(self, value):
        if (value >= 0):
            data = (value * self._shunt_ohms) / self.get_shunt_conv_factor()
        else:
            value_temp = value * (-1)
            data = (value_temp * self._shunt_ohms) / self.get_shunt_conv_factor()
            data = ~data
            data = data + 1
        self.read_register16(self.__INA228_SOVL)
        self.write_register16(self.__INA228_SOVL, data)
        
    def set_shunt_undervoltage(self, value):
        if (value >= 0):
            data = (value * self._shunt_ohms) / self.get_shunt_conv_factor()
        else:
            value_temp = value * (-1)
            data = (value_temp * self._shunt_ohms) / self.get_shunt_conv_factor()
            data = ~data
            data = data + 1

        self.read_register16(self.__INA228_SUVL)
        self.write_register16(self.__INA228_SUVL, data)

    def set_bus_overvoltage(self, value):
        data = value / (16 * 195.3125e-6)
        self.read_register16(self.__INA228_BOVL)
        self.write_register16(self.__INA228_BOVL, data)

    def set_bus_undervoltage(self, value):
        data = value / (16 * 195.3125e-6)
        self.read_register16(self.__INA228_BUVL)
        self.write_register16(self.__INA228_BUVL, data)

    def set_temp_limit(self, value, consta):
        data = value / (16 * consta)
        self.read_register16(self.__INA228_TEMP_LIMIT)
        self.write_register16(self.__INA228_TEMP_LIMIT, data)

    def set_power_overlimit(self, value, consta):
        data = value / (16 * consta)
        self.read_register16(self.__INA228_PWR_LIMIT)
        self.write_register16(self.__INA228_PWR_LIMIT, data)

    def print_manufacturer_id(self):
        raw_id = self.read_register16(self.__INA228_MANUFACTURER_ID)
        print('Manufacturer ID (HEX): ', hex(raw_id))
        first_byte = (raw_id >> 8) & 0xFF
        second_byte = (raw_id & 0xFF)
        print('Manufacturer ID (CHAR): ', chr(first_byte),chr(second_byte))
    
    def print_device_id(self):
        raw_id = self.read_register16(self.__INA228_DEVICE_ID)
        print('Device ID: ', hex(raw_id >> 4))
        print('Revision: ',  hex(raw_id & 0xF))


if __name__ == '__main__':
    import time
    from machine import Pin, I2C

    i2c = I2C(0, scl=Pin(17), sda=Pin(16))
    devices = i2c.scan()
    ina228 = INA228(i2c, address=devices[0], shunt_ohms=0.015)

    ina228.configure()

    ina228.print_manufacturer_id()
    ina228.print_device_id()

    i = 0    

    while True:    
        ina228.print_vbus()
        ina228.print_current()
        ina228.print_shunt_voltage()
        ina228.print_power()
        ina228.print_temp()
        ina228.print_charge()

        if i < 10:
            i = i +1
            print('\n==> i =',i, ' --- ticks_ms =',time.ticks_ms())

        else:
            break
