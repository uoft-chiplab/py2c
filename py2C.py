# py2C project: a comprehensive modules for i2c interfaced devices.
import time 
import smbus

# --- Some constants:
#     device class constants
DEV_SWITCH = 0
DEV_MEAS = 1
DEV_ADC = 2
DEV_DAC = 3

# --- Miscellaneous bit and byte manipulation

def array2bin(array):
    """ Takes an array of ones and zeros and interprets the contents as a 
    binary number. Returns the corresponding integer. """
    out = 0
    for a in array: out = (out<<1) + min(max(int(a),0),1)
    return(int(out))

def int2MSbLSb(num):
    """ Splits a sixteen-bit integer into two 8-bit values (most and least
    significant bytes.) """
    return (bit_grab(num,8,8),bit_grab(num,0,8))

def MSbLSb2int(MSb,LSb):
    """ Returns the 16bit integer represented by a most-dignificant byte MSb 
    and a least-significant byte LSb."""
    return (MSb << 8) + LSb

def byte2bits(num,n=8):
    """ Returns an array of 1 and 0, representing the individual bits of num.
    LSB is byte2bits(..)[0], and 'n' specifies the nbumber of bits (default:
    8, i.e. one byte). Example: 'byte2bits(34)' returns [0,1,0,0,0,1,0,0];
    i.e. bin(34) = 0b100010 """
    return [bit_grab(num,i,1) for i in range(0,n)]

def bit_grab(num,start,nbits):
    """ Returns the value of 'nbit' bits, starting at 'start' (count from LSB).
    E.g. 'bit_grab(0b10111,2,3) returns 5, aka 0b101. """
    return (2**nbits-1)&(num>>start)

def bit_set(num,start,nbits,value):
    """ Returns a modified 'byte', where 'nbits' bits from 'start' have been 
    replaced by 'value' """
    return num + (value<<start) - (num&((2**nbits-1)<<start))

def twoscompl2int(num,n=8):
    """ Returns the interger value represented as two's complement in 'num',
    assuming 'n' bits. """
    return (num&(2**(n-1)-1)) - (num & (~(2**(n-1)-1)))

def bytes2int(byte_array):
    """ Converts a byte_array into a single integer value, assuming ordering
    from MSb (first) to LSb (last) and one's complement. """
    out = byte_array[0]
    for i in range(1,len(byte_array)):
        out = (out << 8) + byte_array[i]
    return out

def int2bytes(value,nbytes=1):
    """ Returns an array of bytes [MSb, ... ,LSb] representing the integer
    'value' as one's complementy. """
    return [(value&(0xff<<(nbytes-i-1)*8))>>((nbytes-i-1)*8)
            for i in range(0,nbytes)]


# ---------- GENERIC I2C DEVICE ----------
class I2c_device(object):
    """ API for generic i2c devices; provides routines for setting class 
    parameters, reading and writing to the device and setting the address. """

    # Some default attributes and initializations
    _dev_type = "generic i2c device" # a string representation of device type
    _dev_class = None # an indicator of the devices general class (e.g. ADC)
    _valid_addr = list(range(0,0b1111111)) # all valid addresses for the device
    # defaults attributes; ! must at least contain bus and address !
    _default = {\
        'addr':0x00,\
        'bus':smbus.SMBus(1),\
    } 
    _bus = None # write-once storage for device's bus
    _addr = None # write-once storage for device's address
    _config = {} # storage place for device's configuration
    
    # configuration register dictionary, specify entries as tuples
    #   (register-address,start-bit,nbits,info,value-representation)
    # needs the additional field 'nbytes', specifying the number of bytes per
    # entry (consult datasheet)
    _conf_reg = {}

    # data register dictionary, specify entries as tuples
    #   (register-addresses,nbytes,)
    _data_reg = {}

    # getter and setter methods for bus and address (locked once set)
    @property
    def bus(self):
        """ The bus on which the device is located. """
        return self._bus
    @bus.setter
    def bus(self,value):
        if self._bus == None:
            self._bus = value
        else:
            raise AttributeError("Cannot change bus once set!")

    @property
    def addr(self):
        """ The 7-bit address of the device. """
        return self._addr
    @addr.setter
    def addr(self,value):
        if self._addr == None:
            self._addr = value
        else:
            raise AttributeError("Cannot change address once set!")

    @property
    def dev_type(self): return self._dev_type
    @property
    def dev_class(self): return self._dev_class

    def __init__(self,read_config=False,**kwargs):
        """ I2C_Device initialization routine. Set 'read_config=False' if the
        device is not ready to use at this time. """
        # pick addr & bus from kwargs (or default) and set
        for kw in ('addr','bus',):
            default = self._default[kw]
            if kw in kwargs:
                setattr(self,kw,kwargs.pop(kw))
            else:
                setattr(self,kw,default)
        # assign other attributes (restricted to those defined in default)
        for kw in self._default:
            if kw not in ('addr','bus',):
                if kw in kwargs:
                    setattr(self,kw,kwargs.pop(kw))
                else:
                    setattr(self,kw,self._default[kw])
        if len(kwargs) > 0:
            print("Ignoring unknown attributes ({})!".format(kwwargs))
        # ready to use; read and store current configuration if requested
        if read_config:
            self._config = self.get_config()
        else:
            for kw in self._conf_reg:
                self._config[kw] = None

    def __str__(self):
        """ A string representation of the device (type @ address) """
        return self._dev_type + " at 0x{0:02X}".format(self._addr)

    def write(self,data=None,ctrl=None):
        """ Generic method for writing 'data' to an i2c device's (control)
        register 'ctrl'. Uses the functinonality provided by smbus. Different 
        devices may have different architectures; sometimes no 'ctrl' is needed
        and sometimes, it is enough to just ping the address. """
        if ctrl is None:
            if data is None:
                # write a zero byte -- used to request a measurement
                # from some devices 
                self.bus.write_byte(self.addr,0x00)    
            else:
                assert type(data) is int and data >= 0 and data < 2**8
                # write data to a device without specifying a control
                # byte, register pointer, or else
                self.bus.write_byte(self.addr,data)
        else:
            assert type(ctrl) is int and ctrl >= 0 and ctrl < 2**8
            if data is None:
                # write only a control byte to the device
                self.bus.write_byte(self.addr,ctrl)
            else:
                # write control byte followed by data byte(s)
                if type(data) is list:
                    for x in data: assert type(x) is int and x >= 0 and x < 2**8
                    self.bus.write_i2c_block_data(self.addr,ctrl,data)
                else:
                    assert type(data) is int and data >= 0 and data < 2**8    
                    self.bus.write_i2c_block_data(self.addr,ctrl,[data])

    def read(self,ctrl=None,nbytes=1):
        """ Generic method for reading 'nbytes' bytes of data from an i2c 
        device's register 'ctrl'. Uses the functinonality provided by smbus. 
        Different devices have different architectures; consult datasheet. 
        Returns the data in the form it was received. """
        nbytes = int(nbytes)
        assert nbytes > 0
        if nbytes > 1:
            if ctrl is None: ctrl = 0x00
            assert type(ctrl) is int and ctrl >= 0 and ctrl < 2**8
            # read nbytes bytes from device
            data = self.bus.read_i2c_block_data(self.addr,ctrl,nbytes)
        else:
            if ctrl is None:
                # read a single byte without sending a control byte
                data = self.bus.read_byte(self.addr)
            else:
                # read a single byte in same way as multiple bytes
                data = self.bus.read_i2c_block_data(self.addr,ctrl,nbytes)
        return data

    def get_config(self,read=True,*args):
        """ Reads the respective parts of the configuration register that 
        contain the properties specified by string inputs. Returns a dictionary
        with one entry for each string input. Returns full configuration if no
        input arguments are given. Setting 'read=False' returns the configuation
        stored in '._config' and throw an error if the requested entry has not
        yet been stored. """
        # return empty dictionary if no configuration register exists
        if len(self._conf_reg) == 0:
            return {}
        # use stored configuration (may be stale)
        if not read:
            out = {}
            for kw in args:
                assert self._config[kw] != None,\
                       "Configuration has to be read from device once!"
                out[kw] = self._config[kw]
            return out
        # read full configuration if prompted without argument
        if len(args) == 0:
            out = {}
            # first collect all configuration register pointers (skip
            # over 'nbytes' entry)
            regs = {self._conf_reg[kw][0] for kw in self._conf_reg \
                    if type(self._conf_reg[kw]) is tuple}
            for r in regs:
                # read register entry
                ans = self.read(r,self._conf_reg['nbytes'])
                assert len(ans) == self._conf_reg['nbytes'],\
                       "Unexpected number of bytes in register!"
                # build integer from read bytes and grab configurations
                val = bytes2int(ans)
                for kw in self._conf_reg:
                    if kw == 'nbytes': continue
                    if self._conf_reg[kw][0] == r:
                        out[kw] = bit_grab(val,self._conf_reg[kw][1],\
                                           self._conf_reg[kw][2])
        # read only configuration specified
        else:
            out = {}
            regs = {self._conf_reg[kw][0] for kw in args}
            for r in regs:
                # read register entry
                ans = self.read(r,self._conf_reg['nbytes'])
                assert len(ans) == self._conf_reg['nbytes'],\
                       "Unexpected number of bytes in register!"
                val = bytes2int(ans)
                for kw in args:
                    if self._conf_reg[kw][0] == r:
                        out[kw] = bit_grab(val,self._conf_reg[kw][1],\
                                           self._conf_reg[kw][2])                
        # automatically update stored configuration
        for kw in out:
            self._config[kw] = out[kw]
        # return dictionary
        return out

    def config(self,**kwargs):
        """ Configures the device, using the control register definition stored
        in 'self._ctrl_reg'. Control fieldnames and values are given as keyword 
        arguments; e.g. if a device 'dev' has the property 'MODE', this property
        is set with dev.config(MODE=value). 
        If no arguments are give, returns the full configuration as a dictionary
        by calling 'self.get_config()'. """
        # if no arguments are given, return full configuration
        if len(kwargs) == 0:
            return self.get_config()
        # if no configuration register is implemented, throw error
        assert len(self._conf_reg) > 0,"No configuration register implemented!"
        # check inputs, collect all registers that need to be updated
        regs = []
        for kw in kwargs:
            assert kw in self._conf_reg,"Unknown property, {}!".format(kw)
            if self._conf_reg[kw][0] not in regs:
                regs.append(self._conf_reg[kw][0])
            assert 0<=kwargs[kw]<2**self._conf_reg[kw][2],\
                   "Property value for {} out of range!".format(kw)
        # cycle through registers, set bits, overwrite register content
        for r in regs:
            ans = self.read(r,self._conf_reg['nbytes'])
            val = bytes2int(ans)
            for kw in kwargs:
                if self._conf_reg[kw][0] == r:
                    val = bit_set(val,self._conf_reg[kw][1],\
                                  self._conf_reg[kw][2],kwargs[kw])
            # break up multi-byte registers into single bytes
            val = int2bytes(val,self._conf_reg['nbytes'])
            self.write(ctrl=r,data=val)
        # finally update stored configuration dictionary
        for kw in kwargs:
            self._config[kw] = kwargs[kw]
        return None

    def config_info(self):
        """ Lists the entries of the configuration register. Prints to the
        standard output. Does not read the register, but merely gives a way to
        show some helpful information. """
        if len(self._conf_reg) == 0:
            return
        # define an order
        kws = sorted([kw for kw in self._conf_reg \
                      if type(self._conf_reg[kw]) is tuple])
        # show configuration register entries
        for kw in kws:
            val = self._conf_reg[kw]
            if val[2] == 1:
                print("{} @ 0x{:02X},{}; {}".\
                      format(kw,val[0],val[1],val[3]))
            else:
                print("{} @ 0x{:02X},{}:{}; {}".\
                      format(kw,val[0],val[1]+val[2]-1,val[1],val[3]))

    def get_raw(self,reg_name=None):
        """ Read from a data register specified in 'self._data_reg'. Returns 
        the value returned by self.read as a single (long) integer. Does not
        convert it according to e.g. two's compliment or a calibration. If only 
        one register is defined in '_data_reg', 'reg_name' may be left empty."""
        assert len(self._data_reg) > 0,"No data register(s) implemented!"
        # default: read first specified register
        if reg_name == None:
            assert len(self._data_reg) == 1,"Need to specify data register!"
            reg_name = self._data_reg[0]
        # read data from register and concatenate to long integer
        data = self.read(ctrl=self._data_reg[reg_name][0],\
                         nbytes=self._data_reg[reg_name][1])
        return bytes2int(data)

    def put_raw(self,value,reg_name=None):
        """ Writes 'value' directly to a data register specified in 
        'self._data_reg'. If only one register is defined in '_data_reg',
        'reg_name' may be left empty. """
        assert len(self._data_reg) > 0,"No data register(s) implemented!"
        if reg_name == None:
            assert len(self._data_reg) == 1,"Need to specify data register!"
            reg_name = self._data_reg[0]
        # split long int value to bytes and write to register
        data = int2bytes(value,self._data_reg[reg_name][1])
        self.write(ctrl=self._data_reg[reg_name][0],data=data)

        

# ---------- I2C DEVICES ----------

# ----- ADS1115: Four -channel ADC (16-Bit) with PGA, Texas Instruments -----
class ADS1115(I2c_device):
    """ This class provides the i2c interface to a ADS1115 chip. Consult 
    datasheet for details on ratings and programming. (ST-2016-06)"""
    # mind offsets and non-linearities for sensitive applications!

    # defining attributes
    BIT_DEPTH = 16
    _dev_type = 'ADS1115'
    _dev_class = DEV_ADC
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {\
        'bus':smbus.SMBus(1), \
        'addr':0x48,\
        'cycle':None,\
        'group':None,\
    }
    # MUX settings for measuring the AIN0-3 vs GND.
    CH0 = 0b100
    CH1 = 0b101
    CH2 = 0b110
    CH3 = 0b111

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'MUX':(0x01,12,3,'MUX setting',\
               ["AIN0-AIN1","AIN0-AIN3","AIN1-AIN3","AIN2-AIN3",\
                "AIN0-GND","AIN1-GND","AIN2-GND","AIN3-GND"]),\
        'PGA':(0x01,9,3,'PGA setting',\
               [6.144,4.096,2.048,1.024,0.512,0.256,0.256,0.256]),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[8,16,32,64,128,250,475,860]),\
        'COMP_MODE':(0x01,4,1,'Comparator mode',[0,1]),\
        'COMP_POL':(0x01,3,1,'Alert-pin polarity',[0,1]),\
        'COMP_LAT':(0x01,2,1,'Comparator latch',[0,1]),\
        'COMP_QUE':(0x01,0,2,'Comp. queiung',[1,2,4,"OFF"]),
    }
    
    # Data registers (3 x 16 bit); CONVersion, LOw THreshold, HIgh THreshold;
    # see datasheet
    ## currently not implemendet: using LOTH and HITH and the COMP feature
    _data_reg = {\
        'CONV':(0x00,2,),\
        'LOTH':(0x10,2,),\
        'HITH':(0x11,2,),\
    }
    
    # methods
    def __init__(self,**kwargs):
        """ Initialize instance. """
        I2c_device.__init__(self,**kwargs)
        
    def put_raw(self,value,reg_name=None):
        """ Do not allow for setting the conversion register. """
        # not really necessary, just an example
        assert reg_name is not "CONV","Cannot set conversion register!"
        I2c_device.put_raw(self,value,reg_name)

    def get_conversion(self):
        """ Reads the conversion register of the chip. Requires a conversion
        request before reading or continuous conversion mode. """
        # get full-scale from (stored) PGA setting
        i = self._config['PGA']
        if i == None: i = self.get_config()['PGA']
        FS = self._conf_reg['PGA'][4][i]
        # read register and return converted voltage reading
        
        return FS*twoscompl2int(self.get_raw('CONV'),16)/(2**15)
    
    def request_conversion(self,ch=None):
        """ Triggers a single-shot conversion by setting the OS bit to 1. 
        Optionally specifying a 'channel' 'ch' through the respective MUX
        setting is possible. """
        if (ch == None) or \
           (ch == 0 and self._dev_type not in ('ADS1115','ADS1015')):
            self.config(OS=0b1)  # trigger conversion
        else:
            # looking ahead
            assert self._dev_type in ('ADS1115','ADS1015'),\
                   'Chip not equipped with multiplexer.'
            assert ch in range(2**self._conf_reg['MUX'][2])
            self.config(MUX=ch,OS=0b1) # set MUX and trigger conversion
    
    def get_single(self,MUX=None,ch=None):
        """ Sets the conversion mode to 1 (SNGL) and reads the conversion \
        register of the chip after requesting a conversion. If 'ch' is \
        specified, sets the MUX to measure AINch vs GND. Alternatively, \
        can set 'MUX' directly (overrides 'ch'). If neither 'ch' nor 'MUX'\
        are set, reads with the current settings."""
        if ch == None and MUX == None:
            # set MODE to SNGL and trigger conversion
            self.config(MODE=0b1,OS=0b1) 
        elif MUX != None:
            # looking ahead to 1114, 1113, 1014, 1013
            if self._dev_type not in ('ADS1115','ADS1015',):
                raise NotImplementedError(\
                    'Chip not equipped with multiplexer.')
            # set MUX, set MODE to SNGL and trigger conversion
            assert MUX in range(0b000,0b111+1),\
                   "MUX setting needs to be in [0b000,0b111]!"
            self.config(MUX=MUX,MODE=0b1,OS=0b1)
        else:
            if (self._dev_type not in ('ADS1115','ADS1015',) and ch > 0) \
               or ch > 3:
                raise NotImplementedError(\
                    'Channel {} does not exist!'.format(ch))
            # set MUX, set MODE to SNGL and trigger conversion            
            self.config(MUX=0b100+ch,MODE=0b1,OS=0b1)
        # wait until conversion is finished, then read
        #print(self.get_config('OS'))
        time.sleep(.01)
        #while not self.get_config('OS'):
        #    pass
        return(self.get_conversion())
    
    def start_continuous(self,ch=None,MUX=None):
        """ Sets the conversion mode to 0 (CONT) for continuous conversion.\
        If 'ch' is specified, sets the MUX to measure AINch vs GND. \
        Alternatively, can set 'mux' directly (overrides 'ch'). If neither is\
        specified, will read with current settings."""
        if ch == None and MUX == None:
            # set MODE to CONT
            self.config(MODE=0b0) 
        elif MUX != None:
            if self._dev_type not in ('ADS1115','ADS1015',):
                raise NotImplementedError(\
                    'Chip not equipped with multiplexer.')
            # set MUX, set MODE to SNGL and trigger conversion
            assert MUX in range(0b000,0b111+1),\
                   "MUX setting needs to be in [0b000,0b111]!"
            self.config(MUX=MUX,MODE=0b0)
        else:
            if (self._dev_type not in ('ADS1115','ADS1015',) and ch > 0) \
               or ch > 3:
                raise NotImplementedError(\
                    'Channel {} does not exist!'.format(ch))
            # set MUX, set MODE to SNGL and trigger conversion            
            self.config(MUX=0b100+ch,MODE=0b0)
    def set_focus(self):
        if self.group == None:
            pass
        else:
            sw = self.group['switch'].get_settings()
            for ch in self.group['channels']:
                sw[ch] = 0
            sw[self.group['me']] = 1
            self.group['switch'].set_channels(sw)


    def get(self):
        """ Short-hand for getting a single conversion from the device. Note
        that this method will set conversion mode to single-shot. """
        self.set_focus()
        if self.cycle == None:
            # read a single valu
            #out = 0
            out = self.get_single()
        else:
            # read, setting MUX to next in cycle; advance cyclo
            
            out = self.get_single(MUX=self.cycle[0])
            self.cycle.append(self.cycle.pop(0))
        self.group['switch'].disable(self.group['me'])
            #print(out)
            #return out
        # return value
        
        
        return out            
    
# ----- ADS1114: Single-channel ADC (16-Bit) with PGA, Texas Instruments -----
class ADS1114(ADS1115):
    """ This class provides the i2c interface to a ADS1115 chip. Consult 
    datasheet for details on ratings and programming. (ST-2016-06)"""
    # mind offsets and non-linearities for sensitive applications!
    ## @@ UNTESTED! Should work just the same as ASD1115

    BIT_DEPTH = 16
    _dev_type = 'ADS1114'
    _dev_class = DEV_ADC
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {
        'bus':smbus.SMBus(1), \
        'addr':0x48, \
        'cycle':None,\
    }

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'PGA':(0x01,9,3,'PGA setting',\
               [6.144,4.096,2.048,1.024,0.512,0.256,0.256,0.256]),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[8,16,32,64,128,250,475,860]),\
        'COMP_MODE':(0x01,4,1,'Comparator mode',[0,1]),\
        'COMP_POL':(0x01,3,1,'Alert-pin polarity',[0,1]),\
        'COMP_LAT':(0x01,2,1,'Comparator latch',[0,1]),\
        'COMP_QUE':(0x01,0,2,'Comp. queiung',[1,2,4,"OFF"]),
    }
    
    # Data registers (3 x 16 bit); CONVersion, LOw THreshold, HIgh THreshold;
    # see datasheet
    _data_reg = {\
        'CONV':(0x00,2,),\
        'LOTH':(0x10,2,),\
        'HITH':(0x11,2,),\
    }

# ----- ADS1113: Single-channel ADC (16-Bit), Texas Instruments -----
class ADS1113(ADS1115):
    """ This class provides the i2c interface to a ADS1115 chip. Consult 
    datasheet for details on ratings and programming. (ST-2016-06)"""
    # mind offsets and non-linearities for sensitive applications!
    ## @@ UNTESTED! Should work just the same as ASD1115

    BIT_DEPTH = 16
    _dev_type = 'ADS1114'
    _dev_class = DEV_ADC
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {
        'bus':smbus.SMBus(1), \
        'addr':0x48,\
        'cycle':None,\
    }

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[8,16,32,64,128,250,475,860]),\
    }
    
    # Data registers (1 x 16 bit); CONVersion, LOw THreshold, HIgh THreshold;
    # see datasheet
    _data_reg = {\
        'CONV':(0x00,2,),\
    }    
    
# ----- ADS1015: Four-channel ADC (12-Bit), Texas Instruments -----
class ADS1015(ADS1115):
    """ The ADS1015 (12-bit version of the ADS1115) is controlled in the 
    same way as the ADS1115. Since the transmitted dataformat is 16-bit, 
    with the last four LSBs set to zero, the same conversion as in the 
    16-bit case can be used. (ST-2016-07)"""

    BIT_DEPTH = 12
    _dev_type = 'ADS1015'
    _dev_class = DEV_ADC
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {
        'bus':smbus.SMBus(1), \
        'addr':0x48,\
        'cycle':None,\
    }

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'MUX':(0x01,12,3,'MUX setting',\
               ["AIN0-AIN1","AIN0-AIN3","AIN1-AIN3","AIN2-AIN3",\
                "AIN0-GND","AIN1-GND","AIN2-GND","AIN3-GND"]),\
        'PGA':(0x01,9,3,'PGA setting',\
               [6.144,4.096,2.048,1.024,0.512,0.256,0.256,0.256]),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[128,250,490,920,1600,2400,3300]),\
        'COMP_MODE':(0x01,4,1,'Comparator mode',[0,1]),\
        'COMP_POL':(0x01,3,1,'Alert-pin polarity',[0,1]),\
        'COMP_LAT':(0x01,2,1,'Comparator latch',[0,1]),\
        'COMP_QUE':(0x01,0,2,'Comp. queuing',[1,2,4,"OFF"]),
    }    
    # otherwise works the same as ADS1115 (last 4 bits of effective 16-bit
    # conversion register are 0000).
  
# ----- ADS1014: Single-channel ADC (12-Bit) with PGA, Texas Instruments -----    
class ADS1014(ADS1115):
    """ This class provides the i2c interface to a ADS1014 chip. Consult 
    datasheet for details on ratings and programming. (ST-2016-06)"""
    # mind offsets and non-linearities for sensitive applications!
    ## @@ UNTESTED! Should work just the same as ASD1115

    BIT_DEPTH = 12
    _dev_type = 'ADS1014'
    _dev_class = DEV_ADC
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {
        'bus':smbus.SMBus(1), \
        'addr':0x48,\
        'cycle':None
    }

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'PGA':(0x01,9,3,'PGA setting',\
               [6.144,4.096,2.048,1.024,0.512,0.256,0.256,0.256]),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[128,250,490,920,1600,2400,3300]),\
        'COMP_MODE':(0x01,4,1,'Comparator mode',[0,1]),\
        'COMP_POL':(0x01,3,1,'Alert-pin polarity',[0,1]),\
        'COMP_LAT':(0x01,2,1,'Comparator latch',[0,1]),\
        'COMP_QUE':(0x01,0,2,'Comp. queiung',[1,2,4,"OFF"]),
    }
    
    # Data registers (3 x 16 bit); CONVersion, LOw THreshold, HIgh THreshold;
    # see datasheet
    _data_reg = {\
        'CONV':(0x00,2,),\
        'LOTH':(0x10,2,),\
        'HITH':(0x11,2,),\
    }



# ----- ADS1013: Single-channel ADC (12-Bit), Texas Instruments -----
class ADS1013(ADS1115):
    """ This class provides the i2c interface to a ADS1115 chip. Consult 
    datasheet for details on ratings and programming. (ST-2016-06)"""
    # mind offsets and non-linearities for sensitive applications!
    ## @@ UNTESTED!

    BIT_DEPTH = 16
    _dev_class = DEV_ADC
    _dev_type = 'ADS1013'
    _valid_addr = [0x48,0x49,0x4a,0x4b]
    _default = {
        'bus':smbus.SMBus(1), \
        'addr':0x48,\
        'cycle':None,\
    }

    # Configuration register (1 x 16bit); see datasheet
    _conf_reg = {\
        'nbytes':2,\
        'OS':(0x01,15,1,'Operative status',['CONV','IDLE']),\
        'MODE':(0x01,8,1,'Conversion mode',["CONT","SNGL"]),\
        'DR':(0x01,5,3,'Data rate',[128,250,490,920,1600,2400,3300]),\
    }
    
    # Data registers (1 x 16 bit); CONVersion, LOw THreshold, HIgh THreshold;
    # see datasheet
    _data_reg = {\
        'CONV':(0x00,2,),\
    }


   
# ----- LSM9DS1_MAG: iNEMO interial module: 3D magnetoimeter, ST -----
class LSM9DS1_MAG(I2c_device):
    """ The iNEMO (LSM9DS1) is a 9DOF sensor that with I2C and SPI interfaces.
    Two addresses are used; one for the magnetometer and one for gyroscope and
    accelerometer. Register architecture is 8-bit. For the 16-bit resolution,
    this means that two registers are used for each measurement. See datasheet 
    for details. (ST-2017-03)"""

    _dev_type = 'LSM9DS1-MAG'
    _dev_class = DEV_MEAS
    _valid_addr = [0x1c,0x1e]
    _default = {\
        'bus':smbus.SMBus(1),\
        'addr':0x1e,\
        'cycle':None,\
        'axis':0,\
        }
    
    # directions of measurement 
    AX_X = 0
    AX_Y = 1
    AX_Z = 2
    
    # Configuration registers (5 x 8Bit), see datasheet
    # combined with status registers
    _conf_reg = {\
        'nbytes':1,\
        'TEMP_COMP':(0x20,7,1,'Temperature compensation',[0,1]),\
        'OMxy':(0x20,5,2,'Operative mode (xy)',["LPow","LP","MP","HP"]),\
        'ODR':(0x20,2,3,'Output data rate',\
               [0.625,1.25,2.5,5.0,10.0,20.0,40.0,80.0]),\
        'ST':(0x20,0,1,'Self test',[0,1]),\
        'FS':(0x21,5,2,'Full scale',[4.0,8.0,12.0,16.0]),\
        'REBOOT':(0x21,3,1,'Reboot memory content',[0,1]),\
        'SOFT_RST':(0x21,2,1,'Register reset function',[0,1]),\
        'I2C_DISABLE':(0x22,7,1,'Disable I2C interface',[0,1]),\
        'LP':(0x22,3,1,'Low-power mode',[0,1]),\
        'SIM':(0x22,2,1,'SPI mode selection',["W","RW"]),\
        'MD':(0x22,0,2,'Operating mode selection',\
              ["CONT","SNGL","POWD","POWD"]),\
        'OMz':(0x23,2,2,'Operative mode (z)',["LPow","LP","MP","HP"]),\
        'BLE':(0x23,1,1,'Big/Little endian',["LSb","MSb"]),\
        'BDU':(0x24,6,1,'Block data update',["CONT","WAIT"]),\
        'ZYXOR':(0x27,7,1,'X,Y and Z overrun'),\
        'ZOR':(0x27,6,1,'Z overrun'),\
        'YOR':(0x27,5,1,'Y overrun'),\
        'XOR':(0x27,4,1,'X overrun'),\
        'ZYXDA':(0x27,3,1,'X,Y and Z data available'),\
        'ZDA':(0x27,2,1,'Z data available'),\
        'YDA':(0x27,1,1,'Y data available'),\
        'XDA':(0x27,0,1,'X data available'),\
    }

    # Data register (6 x 8Bit, see datasheet)
    _data_reg = {\
        'XLO':(0x28,1,), 'XHI':(0x29,1,),\
        'YLO':(0x2a,1,), 'YHI':(0x2b,1,),\
        'ZLO':(0x2c,1,), 'ZHI':(0x2d,1,),\
    }

    ## @@ Not implemented (yet): comparator interrupts and offsets.

    def __init__(self,**kwargs):
        """ Initialize instance """
        I2c_device.__init__(self,**kwargs)

    def get_output(self,axis=0,FS=None):
        """ Returns the measurement output along 'axis'. Pass the fullscale
        'FS' to slightly speed up the interpreter. """
        if axis == 0:
            # read from x-registers
            LSb = self.get_raw('XLO')
            MSb = self.get_raw('XHI')
        elif axis == 1:
            # read from y-registers
            LSb = self.get_raw('YLO')
            MSb = self.get_raw('YHI')
        elif axis == 2:
            # read from z-registers
            LSb = self.get_raw('ZLO')
            MSb = self.get_raw('ZHI')
        # return properly scaled measurement
        if FS == None:
            return self._conf_reg['FS'][4][self.get_config('FS')['FS']]\
                   *twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2**15)
        else:
            return FS*twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2**15)

    def get(self):
        """ Short-hand for getting a single measurement from the device. """
        if self.cycle == None:
            # get measurement along self.axis
            axis = int(self.axis)%3
        else:
            # get measurement along next axis in cycle; advance cycle
            axis = self.cycle[0]
            self.cycle.append(self.cycle.pop(0))
        # return value
        return self.get_output(axis)
    
    
# ----- LSM9DS1_ACC: iNEMO interial module: 3D accelerometer, ST -----
class LSM9DS1_ACC(I2c_device):
    """ The iNEMO (LSM9DS1) is a 9DOF sensor that with I2C and SPI interfaces.
    Two addresses are used; one for the magnetometer and one for gyroscope and
    accelerometer. Register architecture is 8-bit. For the 16-bit resolution,
    this means that two registers are used for each measurement. See datasheet 
    for details. (ST-2017-03)"""

    _dev_type = 'LSM9DS1-ACC'
    _dev_class = DEV_MEAS
    _valid_addr = [0x6a,0x6b]
    _default = {\
        'bus':smbus.SMBus(1),\
        'addr':0x6b,\
        'cycle':None,\
        'mspec':10,\
        }
    
    # directions of measurement 
    AX_X = 0
    AX_Y = 1
    AX_Z = 2
    # measurement type
    GYR = 0
    ACC = 10
    TMP = 20
    
    # Configuration registers, combined with status registers;
    # there is a lot going on here, since many of the settings depend
    # on others (see tables in datasheet; section 7)
    _conf_reg = {\
        'nbytes':1,\
        'TEMP_COMP':(0x20,7,1,'Temperature compensation',[0,1]),\
        'ODR_G':(0x10,5,3,'Gyro output rate',\
                 ['PD',14.9,59.5,119,238,476,952,None]),\
        'FS_G':(0x10,3,2,'Gyro fullscale',range(2**2)),\
        'BW_G':(0x10,0,2,'Gyro bandwidth',[33,40,58,100]),\
        'INT_SEL':(0x11,2,2,'Gyro interrupt selection',[0,1,2,3]),\
        'OUT_SEL':(0x11,0,2,'Gyro output selection',[0,1,2,3]),\
        'LP_mode':(0x12,7,1,'Gyro Low-power mode',[0,1]),\
        'HP_EN':(0x12,6,1,'Gyro High-pass filter enable',[0,1]),\
        'HPCF_G':(0x12,0,4,'Gyro high-pass cutoff selection',range(2**4)),\
        'SignX_G':(0x13,5,1,'X angular rate sign',[+1,-1]),\
        'SignY_G':(0x13,4,1,'Y angular rate sign',[+1,-1]),\
        'SignZ_G':(0x13,3,1,'Z angular rate sign',[+1,-1]),\
        'Orient':(0x13,0,3,'Orientation selection',range(2**3)),\
        'IG_XL':(0x17,6,1,'Accelerometer interrupt',[0,1]),\
        'IG_G':(0x17,5,1,'Gyroscope interrupt',[0,1]),\
        'INACT':(0x17,4,1,'Inactivity interrupt',[0,1]),\
        'BOOT_STATUS':(0x17,3,1,'Boot-running flag',[0,1]),\
        'TDA':(0x17,2,1,'Temperature data ready',[0,1]),\
        'GDA':(0x17,1,1,'Gyroscope data ready',[0,1]),\
        'XLDA':(0x17,0,1,'Accelerometer data ready',[0,1]),\
        'Zen_G':(0x1e,5,1,'Gyro Z-axis enable',[0,1]),\
        'Yen_G':(0x1e,4,1,'Gyro Y-axis enable',[0,1]),\
        'Xen_G':(0x1e,3,1,'Gyro X-axis enable',[0,1]),\
        'LIR_XL1':(0x1e,1,1,'Latched interrupt',[0,1]),\
        '4D_XL1':(0x1e,0,1,'4DOF interrupt option',[0,1]),\
        'DEC':(0x1f,6,2,'Acc data decimation',[1,2,4,8]),\
        'Zen_XL':(0x1f,5,1,'Acc Z-axis enable',[0,1]),\
        'Yen_XL':(0x1f,4,1,'Acc Y-axis enable',[0,1]),\
        'Xen_XL':(0x1f,3,1,'Acc X-axis enable',[0,1]),\
        'ODR_XL':(0x20,5,3,'Accelerometer output rate',range(2**3)),\
        'FS_XL':(0x20,3,2,'Accelerometer fullscale',[2,16,4,8]),\
        'BW_SCAL_ORD':(0x20,2,1,'Scaling of bw with odr',[0,1]),\
        'BW_XL':(0x20,0,2,'Accelerometer bandwidth',[408,211,105,50]),\
        'HR':(0x21,7,1,'High resolution mode',[0,1]),\
        'DCF':(0x21,5,2,'Acc digital filter',range(2**2)),\
        'FDS':(0x21,2,1,'Filtered data selection',[0,1]),\
        'HPIS1':(0x21,0,1,'High pass enable for interrupt',[0,1]),\
        'BOOT':(0x22,7,1,'Reboot memory',[0,1]),\
        'BDU':(0x22,6,1,'Block data update until read',[0,1]),\
        'H_LACTIVE':(0x22,5,1,'Interrupt pin activation level',['HI','LO']),\
        'PP_OD':(0x22,4,1,'Push-pull/open-drain selection',['PP','OD']),\
        'SIM':(0x22,3,1,'SPI interface model sel',['4W','3W']),\
        'IF_ADD_INC':(0x22,2,1,'Automatic register increment',[0,1]),\
        'BLE':(0x22,1,1,'BLE data selection',['LSB-lower','MSB-lower']),\
        'SW_RESET':(0x22,0,1,'Software reset',[0,1]),\
        'SLEEP_G':(0x23,6,1,'Gyro sleep mode',[0,1]),\
        'FIFO_TEMP_EN':(0x23,4,1,'Temp storage FIFO enable',[0,1]),\
        'DRDY_mask_bit':(0x23,3,1,'Data-available timer enable',[0,1]),\
        'I2C_DISABLE':(0x23,2,1,'I2C interface disable',[0,1]),\
        'FIFO_EN':(0x23,1,1,'FIFO memory enable',[0,1]),\
        'STOP_ON_FTH':(0x23,0,1,'Enable FIFO threshold',[0,1]),\
        'ST_G':(0x24,2,1,'Gyro self test enable',[0,1]),\
        'ST_XL':(0x24,0,1,'Acc self test enable',[0,1]),\
    }

    # Data register (6 x 8Bit, see datasheet)
    _data_reg = {\
        'TMP_LO':(0x15,1,), 'TMP_HI':(0x16,1,),\
        'X_G_LO':(0x18,1,), 'X_G_HI':(0x19,1,),\
        'Y_G_LO':(0x1a,1,), 'Y_G_HI':(0x1b,1,),\
        'Z_G_LO':(0x1c,1,), 'Z_G_HI':(0x1d,1,),\
        'X_XL_LO':(0x28,1,), 'X_XL_HI':(0x29,1,),\
        'Y_XL_LO':(0x2a,1,), 'Y_XL_HI':(0x2b,1,),\
        'Z_XL_LO':(0x2c,1,), 'Z_XL_HI':(0x2d,1,),\
    }

    ## @@ Not implemented (yet): comparator interrupts and offsets.

    def __init__(self,**kwargs):
        """ Initialize instance """
        I2c_device.__init__(self,**kwargs)

    def get_gyro(self,axis=0,FS=None):
        """ Returns the measurement output along 'axis'. Pass the fullscale
        'FS' to slightly speed up the interpreter. """
        if axis == 0: (reg_lo,reg_hi) = ('X_G_LO','X_G_HI')
        elif axis == 1: (reg_lo,reg_hi) = ('Y_G_LO','Y_G_HI')
        elif axis == 2: (reg_lo,reg_hi) = ('Z_G_LO','Z_G_HI')
        (LSb,MSb) = (self.get_raw(reg_lo),self.get_raw(reg_hi))
        # return properly scaled measurement
        if FS == None:
            return twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)
        else:
            return FS*twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)
        
    def get_acc(self,axis=0,FS=None):
        """ Returns the measurement output along 'axis'. Pass the fullscale
        'FS' to slightly speed up the interpreter. """
        if axis == 0: (reg_lo,reg_hi) = ('X_XL_LO','X_XL_HI')
        elif axis == 1: (reg_lo,reg_hi) = ('Y_XL_LO','Y_XL_HI')
        elif axis == 2: (reg_lo,reg_hi) = ('Z_XL_LO','Z_XL_HI')
        (LSb,MSb) = (self.get_raw(reg_lo),self.get_raw(reg_hi))
        # return properly scaled measurement
        if FS == None:
            return 1.0*twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)
        else:
            return FS*twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)
        
    def get_temp(self,FS=None):
        """ Returns the measurement output along 'axis'. Pass the fullscale
        'FS' to slightly speed up the interpreter. """
        (reg_lo,reg_hi) = ('TMP_LO','TMP_HI')
        (LSb,MSb) = (self.get_raw(reg_lo),self.get_raw(reg_hi))
        # return properly scaled measurement
        if FS == None:
            return twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)
        else:
            return FS*twoscompl2int(MSbLSb2int(MSb,LSb),n=16)/(2.0**15)

    def get_output(self,spec,FS=None):
        """ Returns the output specified by SPEC in terms of the class'
        constants (AX_X, AX_Y, AX_Z) + (GYR, ACC, TMP). """
        # break up measurement specification
        axis = spec%10
        mtype = spec - axis
        # check correctness
        assert axis in (self.AX_X,self.AX_Y,self.AX_Z),"Invalid axis spec!"
        assert mtype in (self.GYR,self.ACC,self.TMP),\
               "Invalid measurement spec!"
        # return respective data
        if mtype == self.TMP:
            return self.get_temp(FS=FS)
        elif mtype == self.ACC:
            return self.get_acc(axis=axis,FS=FS)
        elif mtype == self.GYR:
            return self.get_gyro(axis=axis,FS=FS)

    def get(self):
        """ Short-hand for getting a single measurement from the device. """
        if self.cycle == None:
            # get measurement along self.axis
            spec = self.mspec
        else:
            # get measurement along next axis in cycle; advance cycle
            spec = self.cycle[0]
            self.cycle.append(self.cycle.pop(0))
        # return value
        #self.group['switch'].disable(self.group['me'])
        return self.get_output(spec)



# ----- TCA9545A: Four-channel isolating i2c switch, Texas Instruments -----
class TCA9545A(I2c_device):
    """ TI's TCA9545A is a four-channel isolating i2c switch with one interrupt 
    line per channel. A single control byte is used to write/read its settings. 
    (ST-2016-09) ."""
    ## @@ UNTESTED!
    
    ## @@ Not implemented (yet): interrupt support

    _dev_type = 'TCA9545A'
    _dev_class = DEV_SWITCH
    _default = {\
        'bus':smbus.SMBus(1), \
        'addr':0x70,\
        }
    _valid_addr = (0x70,0x71,0x72,0x73,)

    def __init__(self,**kwargs):
        I2c_device.__init__(self,**kwargs)
        
    def config(self,*args,**kwargs): raise NotImplementedError
    def get_config(self,*args,**kwargs): raise NotImplementedError
    def config_info(self,*args,**kwargs): raise NotImplementedError

    def set_channels(self,settings):
        """ Enables/disables channels according to bit xi=1/0 in 
        settings = [x0,x1,x2,x3]."""
        assert type(settings) is list and len(settings) == 4
        # @@ interrupts not implemented for now 
        ctrl_byte = 0b0000
        # mind the order: xxxx3210
        for s in reversed(settings):
            assert s in (0,1),"Channel settings need to be 0 or 1!"
            ctrl_byte = (ctrl_byte << 1) + s
        self.write(ctrl=ctrl_byte)

    def get_settings(self):
        """ Returns the current channel settings [x0,x1,x2,x3]."""
        data = byte2bits(self.read())
        # only return the channel settings (4 LSB), discard interrupts (4 MSB)
        # mind the order: xxxx3210
        return data[0:4]
    
    def enable(self, channels):
        """ Enables one or more channels."""
        if type(channels) is not list: channels = [channels]
        assert max(channels) <= 3; assert min(channels) >= 0
        settings = self.get_settings()
        for ch in channels: settings[ch] = 1
        self.set_channels(settings)

    def disable(self, channels):
        """ Disables one or more channels."""
        if type(channels) is not list: channels = [channels]
        assert max(channels) <= 3; assert min(channels) >= 0
        settings = self.get_settings()
        for ch in channels: settings[ch] = 0
        self.set_channels(settings)

    def enable_all(self):
        """ Enables all channels."""
        self.set_channels([1,1,1,1])

    def disable_all(self):
        """ Disables all channels."""
        self.set_channels([0,0,0,0])


        
# ----- TCA9548A: Eight-channel isolating i2c switch, Texas Instruments -----
class TCA9548A(I2c_device):
    """ TI's TCA9548A is an eight-channel isolating i2c switch with reset. \
    (ST-2017-02) ."""
    
    _dev_type = 'TCA9548A'
    _default = {\
        'bus':smbus.SMBus(1), \
        'addr':0x70,\
        }
    _valid_addr = (0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77)

    def __init__(self,**kwargs):
        I2c_device.__init__(self,**kwargs)
        
    def config(self,*args,**kwargs): raise NotImplementedError
    def get_config(self,*args,**kwargs): raise NotImplementedError
    def config_info(self,*args,**kwargs): raise NotImplementedError

    def set_channels(self,settings):
        """ Enables/disables channels according to bit xi=1/0 in 
        settings = [x0,x1,x2,x3,x4,x5,x6,x7]."""
        assert type(settings) is list and len(settings) == 8
        # @@ interrupts not implemented for now 
        ctrl_byte = 0
        # mind the order: xxxx3210
        for s in reversed(settings):
            assert s in (0,1,),"Channel settings need to be 0 or 1!"
            ctrl_byte = (ctrl_byte << 1) + s
        self.write(ctrl=ctrl_byte)

    def get_settings(self):
        """ Returns the current channel settings [x0,x1,x2,x3,x4,x5,x6,x7]."""
        return byte2bits(self.read())
    
    def enable(self, channels):
        """ Enables one or more channels."""
        if type(channels) is not list: channels = [channels]
        assert max(channels) <= 7; assert min(channels) >= 0
        settings = self.get_settings()
        for ch in channels: settings[ch] = 1
        self.set_channels(settings)

    def disable(self, channels):
        """ Disables one or more channels."""
        if type(channels) is not list: channels = [channels]
        assert max(channels) <= 7; assert min(channels) >= 0
        settings = self.get_settings()
        for ch in channels: settings[ch] = 0
        self.set_channels(settings)

    def enable_all(self):
        """ Enables all channels."""
        self.set_channels([1,1,1,1,1,1,1,1])

    def disable_all(self):
        """ Disables all channels."""
        self.set_channels([0,0,0,0,0,0,0,0])


        
# ----- HIH8121: Humidity and temperature sensor (14-Bit), Honeywell -----
class HIH8121(I2c_device):
    """ This class provides the i2c interface to a Honeywell HIH8182 humidity 
    and temperature sensor. Consult datasheet for details on ratings and 
    programming. (ST-2016-09)"""

    BIT_DEPTH = 14
    _dev_type = 'HIH8121'
    _dev_class = DEV_MEAS
    _valid_addr = [0x27]
    _default = {\
        'bus':smbus.SMBus(1), \
        'addr':0x27, \
        'hum_range':100.0, \
        'hum_offset':0.0, \
        'temp_range':165.0 ,\
        'temp_offset':-40.0,\
        'group':None,\
        'cycle':None,\
        'get_temp':0,\
        }
    # addressing humidity and temperature data
    HUM = 0
    TMP = 1

    def __init__(self,**kwargs):
        """ Initialize instance """
        I2c_device.__init__(self,**kwargs)

    # architecture without registers
    def config(self,*args,**kwargs): raise NotImplementedError
    def get_config(self,*args,**kwargs): raise NotImplementedError
    def config_info(self,*args,**kwargs): raise NotImplementedError

    def request_measurement(self):
        """ Requests (triggers) a measurement.
        Warning: does not set the focus to this sensor if part of a group! """
        self.write()

    def get_data(self):
        """ Read humidity and temperature data from the chip. Returns a tuple
        (humidity,temperature,status). Will read stale data if no new
        measurements are requested.
        Warning: does not set the focus to this sensor if part of a group! """
        data = self.read(ctrl=0x00,nbytes=4)
        # first two bits give the chip's status 0b00 for normal; 0b01 for stale data
        status = data[0]>>6 
        # next 6+8 bits are the humidity data
        hum_raw = ((data[0]-(status<<6)) << 8) + data[1]
        humidity = self.hum_range*hum_raw/(2**self.BIT_DEPTH-2) \
                   + self.hum_offset
        # next 8+6 bits are the temperature data; last two bits do not matter
        temp_raw = (data[2] << 8)+data[3] >> 2
        temperature = self.temp_range*temp_raw/(2**self.BIT_DEPTH-2) \
                      + self.temp_offset
        return(humidity,temperature,status)
    
    def set_focus(self):
        """ Sets the focus on this  sensor, if it is part of a group. This is 
        done by choosing the switch settings that exclusively targets this 
        HIH sensor, muting all others in the group. """
        if self.group == None:
            # do nothing, if not part of a group
            pass
        else:
            # build switch settings
            sw = self.group['switch'].get_settings()
            for ch in self.group['channels']:
                sw[ch] = 0
            sw[self.group['me']] = 1
            # set switch to disable all channels in group except for 'mine'
            self.group['switch'].set_channels(sw)
            

    def get(self):
        """ Short-hand for getting a single measurement from the device. """
        # set focus to 'me' if part of a group
        self.set_focus()
        # request new measurement and retrieve values
        self.request_measurement()
        data = self.get_data()
        
        if self.cycle == None:
            # @@ not a great name
            i = self.get_temp
        else:
            # get measurement along next axis in cycle; advance cycle
            i = self.cycle[0]
            self.cycle.append(self.cycle.pop(0))
        # return value
        self.group['switch'].disable(self.group['me'])
        return data[i]
    
# ----- HIH8120, 7121, 7120: only differ from HIH8121 in accuracy and
# ----- package (x121 with filter)
class HIH8120(HIH8121):
    _dev_type = 'HIH8120'
class HIH7121(HIH8121):
    _dev_type = 'HIH7121'
class HIH7120(HIH8121):
    _dev_type = 'HIH7120'    
        
    
       
## ------------------- Everything below is under construction! -----------------


# ----- DAC8574: Four-channel DAC (16 Bit), Texas Instruments  -----
class DAC8574(I2c_device):
    """ This class provides the i2c interface to a DAC8574 chip. Consult \
    datasheet for details on ratings and programming. DAC has four address\
    pins -- two setting the i2c address, and two extended addressing bits\
    allowing 64 channels to be controlled through one i2c address. (ST-2016-09)"""
    # mind offsets and non-linearities for sensitive applications!

    BIT_DEPTH = 16
    _dev_type = 'DAC8574'
    _dev_class = DEV_DAC
    _valid_addr = [0x4c,0x4d,0x4e,0x4f]
    _default = {'bus':smbus.SMBus(1), \
               'addr':0x4c, \
               'ext_addr':0b00, \
               'Vref':2.486, \
               'Voff':0.019}
    _ext_addr = None
    
    @property
    def ext_addr(self):
        """ The 2-bit extended address of the device. """
        return self._ext_addr
    @ext_addr.setter
    def ext_addr(self,value):
        if self._ext_addr == None:
            assert value in range(2**2),"Invalid extended address!"
            self._ext_addr = value
        else:
            raise AttributeError("Cannot change extended address once set!")
        
    # data register: addresses -- aka ctrl byte -- depends on the extended
    # address, here initialized as 0b00. The last bit is power-down. Bits
    # 5&4 are Load1 and Load0
    _data_reg = {\
        'TRA':(0b00000000,2,),\
        'TRB':(0b00000010,2,),\
        'TRC':(0b00000100,2,),\
        'TRD':(0b00000110,2,),\
        }
    
    def __init__(self,**kwargs):
        """ Initialize instance """
        I2c_device.__init__(self,**kwargs)
        # update data register ctrl bytes with extended address
        for reg in self._data_reg:
            self._data_reg[reg] = \
                (self._data_reg[reg][0] + (self.ext_addr << 6),\
                 self._data_reg[reg][1],)
            
    # architecture without many registers
    def config(self,*args,**kwargs): raise NotImplementedError
    def get_config(self,*args,**kwargs): raise NotImplementedError
    def config_info(self,*args,**kwargs): raise NotImplementedError
            
    def set_output(self,ch,value,units=None,load=0b01):
        """ Sets the output of channel ch (0-3 = A-D) to value. ch=None can be
        used to simultaneously set all channels on all listening DACs (broadcast
        mode). The default load=0b01 causes the data sent to be loaded in to the 
        DAC upon receiving. Use load=0b00 to store data in register without 
        loading the DAC. Use load=0b10 to load all DACs after receiving. """
        # validate and process inputs
        if ch == None:
            # broadcast data to all
            ctrl = 0b00110100 
        else:
            regs = ('TRA','TRB','TRC','TRD',)
            if type(ch) is str:
                assert ch in regs,"Invalid DAC output channel!"
                ctrl = sel._data_reg[ch]
            else:
                assert ch in range(4),"Invalid DAC output channel!"
                ctrl = self._data_reg[regs[ch]][0]
            assert load in range(3),"Invalid load settings!"
            # shift load bits and add to ctrl
            ctrl += (load << 4)
        # process the input value
        if units == "V": 
            value = (value-self.Voff)/(self.Vref-self.Voff)
        value = int(max(0,min(1,value)) * (2**self.BIT_DEPTH - 1))
        # send data split to two bytes
        data = int2bytes(value,2)
        self.write(data,ctrl)

    #def store_value(self,ch,value,units=None):
        #" Sets the temporary register of the selected DAC to\
        #value, but does not change output, yet. Use ch=-1 to\
        #set all four channels to the same value."
        #if ch==-1:
            ## write to each channel's register
            #for i in range(0,4):
                #self.store_value(i,value,units=units)
        #else:
            #self.write_DAC_data(ch,value,load=[0,0],units=units)
        ## DEBUG -- DESPITE RIGHT CONTROL BIT, OUTPUT VALUE DOES GET
        ##          UPDATED. PROBLEM UNCLEAR ...

    #def update_outputs(self):
        #" Broadcasts the update command to all DACs, causing them\
        #to change output according to stored data. This will affect\
        #all DAC8574s (and equivalent) that share the same i2c address."
        ## write to device
        #self.write(ctrl=array2bin([0,0,1,1,0,0,0,0]))

    #def write_DAC_data(self,ch,value,load=[0,1],units=None):
        #" Writes to the DAC registers, either to store data for a\
        #broadcast update, or for immediate update. load refers to\
        #Load1 and Load0 bits (in this order) selecting the update\
        #mode (see datasheet). Default is an update of the selected\
        #channel with value. For a simultaneous updateof all four\
        #channels, ch=-1 or load=[1,0] should be set."
        #assert ch >= -1 and ch < 4
        #if ch == -1:
            #load = [1,0] # write to all (cannot store this way, though)
            #ch = [0,0] # irrelevant
        #else:
            #ch = [ch%2, ch//2]
        ## convert voltage units to relative value 0..1
        #if units == "V": value = (value-self.Voff)/(self.Vref-self.Voff)
        #if value < 0: value = 0
        #if value > 1: value = 1
        #value = int(value * (2**self.BIT_DEPTH - 1))
        ## build control byte and most/least significant data bytes
        #ctrl_byte = [self.addr_pin[3],self.addr_pin[2],\
                     #load[0],load[1],0,ch[1],ch[0],0]
        #ctrl_byte = array2bin(ctrl_byte)
        #MS_byte, LS_byte = int2MSbLSb(value)
        ## write to device
        #self.write(ctrl=ctrl_byte,data=[MS_byte,LS_byte])
        
    #def power_down(self,ch,*arg):
        #" Powers down DAC channel ch (0-3 = A-D) with selected mode. \
        #0: High-Z out; 1: 1kOhm to gnd; 2: 100kOhm to gnd; 3: High-Z (?) \
        #Default: mode = 0."
        ## The syntax is similar to the one described in self.set_output.
        ## check mode argument; if none given, set default to mode = 0
        #assert len(arg) <= 1
        #if len(arg) == 0: mode = 0
        #else: mode = arg[0]
        
        #if ch == -1:
            ## ch=-1: all channels are powered down
            #for i in range(0,4): self.power_down(i,mode)
        #else:
            ## check and modify channel & mode input
            #assert ch >= 0 and ch < 4
            #ch = [ch%2, ch//2]
            #assert mode >= 0 and mode < 4
            #mode = [mode%2, mode//2]            
            ## build control byte
            #ctrl_byte = [self.addr_pin[3],self.addr_pin[2],0,1,0,ch[1],ch[0],1]
            #ctrl_byte = array2bin(ctrl_byte)
            ## build MSb (power down command); LSb is 0.
            ## The power-down mode is controlled with the first two bits
            #MS_byte = ((mode[1]<<1) + mode[0])<<6
            ## write to device
            #self.write(ctrl=ctrl_byte,data=[MS_byte,0])


## ----- DAC8574 group, broadcast to same address, controlling up to 4 chips  -----
#class DAC8574_Group(I2c_device):
    #" This class provides the i2c interface to a group of DAC8574 chips.\
    #Consult datasheet for details on ratings and programming. DAC has \
    #four address pins -- two setting the i2c address, and two extended \
    #addressing bits allowing 16 channels to be controlled through one \
    #i2c address. (ST-2016-09)"
    ## mind offsets and non-linearities for sensitive applications!

    #BIT_DEPTH = 16
    #TYPE = 'DAC8574 group'
    #VALID_ADDR = [0x4c,0x4d,0x4e,0x4f]
    #DEFAULT = {'bus':smbus.SMBus(1), \
               #'addr_pin':[0,0], \
               #'addr':0x4c, \
               #'addr_mask':"10011ba", \
               #'addr_active':[[0,0]]}

    #def __init__(self,**kwargs):
        ## first initialize with default attribute, then update with kwargs
        #self.setpar(self.DEFAULT)
        #self.setpar(**kwargs)
        #if 'addr_pin' in kwargs:
            #self.set_address(addr_pin=kwargs['addr_pin'])
        #self.DAC = [None,None,None,None]
        #assert type(self.addr_active) is list
        #if type(self.addr_active[0]) is not list:
            #self.addr_active = [self.addr_active]
        #for pin in self.addr_active:
            #assert len(pin) == 2
            #addr_pin = self.addr_pin
            #addr_pin.extend(pin)
            #i = (pin[0]<<1)+pin[1]
            #self.DAC[i] = DAC8574(addr_pin=addr_pin)

    #def set_output(self,ch,value,units=None):
        #" Sets the output of a single channel, where addressing is\
        #4-bit word [A3,A2,Sel1,Sel0], i.e. extended address (0..3)\
        #combined with channel selection (0..3)."
        #if ch == -1:
            #for DAC in self.DAC:
                #if DAC:
                    #DAC.store_value(-1,value,units=units)
        #else:
            ## break up address
            #chip = ch>>2
            #ch -= chip<<2
            ## set output if chip exists in group
            #if self.DAC[chip]:
                #self.DAC.write_DAC_data(ch,value,load=[0,1],units=units)
            #else:
                #raise IOError("Chip {} does not exist in group".format(chip))

    #def store_value(self,ch,value,units=None):
        #" Sets the temporary register of the selected DAC to\
        #value, but does not change output, yet. Use ch=-1 to\
        #set all four channels to the same value."
        #if ch == -1:
            #for DAC in self.DAC:
                #if DAC:
                    ## write to each channel's register
                    #for i in range(0,4):
                        #DAC.store_value(i,value,units=units)
        #else:
            ## break up address
            #chip = ch>>2
            #ch -= chip<<2
            ## set output if chip exists in group
            #if self.DAC[chip]:
                #self.DAC.write_DAC_data(ch,value,load=[0,0],units=units)
            #else:
                #raise IOError("Chip {} does not exist in group".format(chip))        

    #def update_outputs(self):
        #" Broadcasts the update command to all DACs, causing them\
        #to change output according to stored data."
        ## broadcast to devices
        #self.write(ctrl=array2bin([0,0,1,1,0,0,0,0]))            
        








    

if __name__ == "__main__":
    # do some testing here
    dac = DAC8574(addr=0x4d)
    print(dac.set_output(1,0.2))
    
    


      






