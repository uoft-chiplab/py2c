# Datalogging with the raspberry pi and i2c devices -- ST 03/2017
#
#   Running on 2.7.9; everything but gpio functionality works in 3.5 as well.
#
import py2C as i2c
import RPi.GPIO as gpio
import numpy as np
import time
import smbus

class DataLogger():
    """ A simple data-to-file logging class. """

    _default = {\
        'meas_period':0.1,\
        'avg_period':1.0,\
        'trigger_pin':None,\
        'trigger_enable':False,\
        'trigger_timeout':10*1000,\
        'path':"./",\
        'filemask':"DataLog_3_{2:04}-{1:02}-{0:02}.txt",\
        }
    
    def __init__(self,**kwargs):
        # pick out device specifications and add devices
        self._devices = []
        if 'devices' in kwargs:
            devices = kwargs.pop('devices')
            for d in devices:
                assert isinstance(d,i2c.I2c_device),\
                       "Expecting instance of I2c_device!"
                assert d.dev_class in (i2c.DEV_MEAS,i2c.DEV_ADC,),\
                       "Unsupported device class for device '{}'!"\
                       .format(d.dev_type)
                ## @@ check for device class as well.
                self._devices.append(d)
        else:
            devices = []
        # first set defaults, then overwrite with possible user input
        for kw in self._default:
            setattr(self,kw,self._default[kw])
        for kw in kwargs:
            assert kw in self._default,\
                   "Uknown keyword '{}!'".format(kw)
            setattr(self,kw,kwargs[kw])
        # initialize data list
        self._data = []

    def add_device(self,device):
        """ Append a new device to the end of the devices list. Note, that
        using the same device multiple times prompts a new measurement every
        time. Use cycling to access different 'channels' in one device. """
        assert isinstance(device,i2c.I2c_device),\
               "Expecting instance of I2c_device!"
        assert d.dev_class in (i2c.DEV_MEAS,i2c.DEV_ADC,),\
               "Unsupported device class for device '{}'!"\
               .format(d.dev_type)
        # append
        self._devices.append(device)

    def get_measurements(self):
        """ Returns the list of measurement values obatained by each device's
        get() method. """
        return [device.get() for device in self._devices]

    def start_measurement_loop(self):
        " Starts the measurement loop for this DataLogger. "
        # @@ cheap and dirty!!
        while True:
            line_note = ""
            triggered = "0"
            # wait for trigger if triggered operation is selected
            if self.trigger_enable and self.trigger_pin != None:
                res = gpio.wait_for_edge(self.trigger_pin,\
                                         gpio.RISING,\
                                         timeout=self.trigger_timeout)
                if res == None:
                    line_note = "timeout"
                else:
                    line_note = "TR({})".format(res)
                    triggered = "1"
            # initialize empty data list
            ## @@ verify that there is no memory leak here (used .clear()
            ## @@ before, which is not supported in 2.7.9)
            self._data = []
            last_avg = time.time()
            while (time.time() - last_avg) < self.avg_period:
                # get a measurement
                last = time.time()
                self._data.append(self.get_measurements())
                # wait for next measurement
                while (time.time() - last) < self.meas_period:
                    pass
            avg = [sum([data[i] for data in self._data])\
                   /len(self._data) \
                   for i in range(0,len(self._data[0]))]
            # build filename with current date
            now = time.localtime()
            outfile = self.path + self.filemask.\
                      format(now.tm_mday,now.tm_mon,now.tm_year)
            # build timestamp
            timestamp = "{:02}:{:02}:{:02}".\
                        format(now.tm_hour,now.tm_min,now.tm_sec)
            # build line
            line = ",".join(["{:.4f}".format(a) for a in avg])
            line += "," + triggered
            # print to stdandard output
            print_line = " , ".join(["{:.4f}".format(a) for a in avg])
            print(outfile + " < " + print_line + "   @ " \
                  + timestamp + "   " + line_note)
            # append to file
            with open(outfile,'a') as f:
                f.write(timestamp + "," +line+"\n")

def triggered_trace(trigger_pin,devices,timeout=-1,tmax=None,nmax=10,\
                    dt=None):
    """ Performs a triggered measurement, accumulating samples either until
    'nmax' samples are reached or until theloop has run for time 'tmax'.
    Optionally can force time intervals of measurements to 'dt'.
    The loop starts after a rising flank has been detected on 'trigger_pin',
    or once the 'timeout' time is elapsed. """
    # reshape and validate input
    if type(devices) not in (tuple,list,):
        devices = [devices]
    assert nmax != 0 or tmax != 0,"Missing break condition!"
    # initialize empty data array
    data = []
    # start by waiting for the trigger
    gpio.wait_for_edge(trigger_pin,gpio.RISING,timeout=timeout)
    print('Go!')
    start=time.clock()
    # based on choices: slightly different loops
    if dt == None:
        if tmax == None:
            # continuous loop until nmax reached
            while len(data) < nmax:
                data.append([time.clock()-start]\
                            +[d.get() for d in devices])
        else:
            # continuous loop until nmax or tmax reached
            while (len(data) < nmax) and (time.clock()-start < tmax):
                data.append([time.clock()-start]\
                            +[d.get() for d in devices])
    else:
        if tmax == None:
            # continuous loop until nmax reached, waiting for dt
            while len(data) < nmax:
                data.append([time.clock()-start]\
                            +[d.get() for d in devices])
                while (time.clock()-start < dt*len(data)):
                    pass
        else:
            # continuous loop until nmax or tmax reached, waiting for dt
            while (len(data) < nmax) and (time.clock()-start < tmax):
                data.append([time.clock()-start]\
                            +[d.get() for d in devices])
                while (time.clock()-start < dt*len(data)):
                    pass
    # hand back the measurement result
    return data
            
if __name__ == "__main__":       
    print('RELEASE THE KRAKEN!!!')
    # setup a trigger pin
    # (BCM indexing; pins on break-out board are 12, 16, 26)
    #gpio.setmode(gpio.BCM)
    
    
    #gpio.setup(16,gpio.IN)

    # setup i2c devices of interest
    # 8-channel i2c bus expander
    tca = i2c.TCA9548A(addr=0x71)
    tca.disable_all()
    #tca2 = i2c.TCA9548A(addr=0x70)
    #tca2.disable_all()
    #tca.disable_all()
    #tca2 = i2c.TCA9548A(addr=0x71)
    #tca2.disable_all()
    # initially enable channels with devices
    #tca2.set_channels([0,0,1,1,0,0,0,0])
    tca.set_channels([1,0,0,0,0,0,0,0])
    #%tca3=i2c.TCA9548A(addr=0x71)
    #tca



    hih_channels = [0,1,2,3,4,5,6,7]
    hih2_channels = [0,1,2,4,5,6,7]
    hih = [i2c.HIH8121(addr=0x27,cycle=[0,1],\
                       group={'me':ch,'channels':hih_channels,\
                              'switch':tca})\
           for ch in hih_channels]
    tca.disable_all()
    tca2 = i2c.TCA9548A(addr=0x70)
    tca2.disable_all()
    tca2.set_channels([0,0,0,1,0,0,0,0])


    hih2 = [i2c.HIH8121(addr=0x27,cycle=[0,1],\
                        group={'me':ch,'channels':hih2_channels,\
                                'switch':tca2})\
             for ch in hih2_channels]
    tca.set_channels([1,1,1,1,1,1,1,1])
    tca.disable_all()
    tca2.disable_all()
    # ADS1015: 4-channel ADC
    #adc_channels = [0,1,2,3]
    adc_channels=[3]

    #adc =i2c.ADS1115(addr=0x48,cycle=[0b100,0b101,0b110,0b111])
    #adc = i2c.ADS1115(addr=0x48)
    adc = [i2c.ADS1115(addr=0x48,cycle=[0b100,0b101,0b110,0b111],\
            group={'me':ch,'channels':adc_channels,\
            'switch':tca2})\
            for ch in adc_channels]
    #ms = i2c.LSM9DS1_MAG(addr=0x1c)
    #ac = i2c.LSM9DS1_ACC(addr=0x6b)
            #adc =i2c.ADS1015(addr=0x4a,cycle=[0b100,0b101,0b110,0b111])
            
            ## Some triggered trace taking ...
    #print('Waiting for trigger ...')
    #tt = triggered_trace(16,[adc,adc,adc,adc],\
             #            dt=0.004,tmax=2.0,nmax=100000)
    #with open("test_trace.txt",'w') as f:
        #for t in tt:
    
    
##    # create a datalogger object for the devices
##    log = DataLogger(filemask="DataLog_Triggered-QPD_{2:04}-{1:02}-{0:02}.txt",\
##                     path="/home/pi/Documents/Data Log/Triggered-QPD/",\
##                     devies=[adc,adc,adc,adc,\
##                              hih[0],hih[0],hih[1],hih[1],hih[2],hih[2]],\
##                     trigger_pin=16,\
##                     trigger_enable=True
  # create a datalogger object for the devices
    log = DataLogger(filemask="DataLog_tempChipLab_{2:04}-{1:02}-{0:02}.txt",\
                     path="/home/pi/Documents/Data Log/",\
                     devices=[hih[0],hih[0],hih2[1],hih2[1],hih2[2],hih2[2],hih[3],hih[3],hih[2],hih[2],hih[0],hih[0],hih[1],hih[1]],\
#,hih[1],hih[1],hih[2],hih[2],hih[3],hih[3],hih[4],hih[4],hih[5],hih[5],hih[6],hih[6],hih[7],hih[7]],\
                     avg_period=5.0)
    
    # start the measurement loop
    try:
        print('Press CTRL-C to exit loop.')
        log.start_measurement_loop()
    except KeyboardInterrupt: 
        print('Goodbye!')
    finally:
        gpio.cleanup()

