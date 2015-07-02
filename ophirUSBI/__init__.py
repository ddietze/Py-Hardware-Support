"""
.. module: ophirUSBI
   :platform: Windows
.. moduleauthor:: Daniel Dietze <daniel.dietze@berkeley.edu>

Python module for accessing Ophir's USB power meters through the OphirUSBI ActiveX interface.

.. note:: Requires *pywin32* and Ophir's USBI ActiveX component (status 2010). Ophir has since released new versions of the ActiveX components and also a newer COM object, neither of which is supported currently. The ActiveX control is included by early-binding. It is necessary to run the *Com - Makepy Utility* found in the *PythonWinIDE - Tools - Menu* in order to make it accessible to python.  

Short example::
    
    import ophirUSBI
    
    # create class instance and connect to ActiveX component
    dev = ophirUSBI.OphirUSBI()     
    
    # connect to first OphirUSBI device
    dev.connect(0)                  
    
    # set measurement mode
    dev.set_default_mode()          
    
    # record 10 data points
    t, y = dev.read_data(10)        
    
    dev.disconnect()

..
   This program is free software: you can redistribute it and/or modify 
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Copyright 2010 Daniel Dietze <daniel.dietze@berkeley.edu>.   
""" 

# modules
import win32com.client
from time import sleep

# class
class OphirUSBI:
    """Python interface to the Ophir USBI device.
    
    """
    
    # constructor
    def __init__(self):
        """Class constructor.
        
        Calling this constructor automatically connects to the ActiveX component.
        """
        self.USBI_handle = 0
        self.USBI_channel = 0       #: Channel number for readout.
        self.timeout = 0.1          #: Timeout for data acquisition using :py:func:`read_data` in seconds.
        self.device_ready = False   # True if device has already delivered data; otherwise or after timeout, this is set to False
        
        self.measurement_mode = 0
        self.MM_Modes = ()          #: List of available measurement modes as returned by :py:func:`connect`.
        self.pulse_length = 0
        self.MM_PulseLengths = ()   #: List of available pulse lengths as returned by :py:func:`connect`.
        self.range = 0
        self.MM_Ranges = ()         #: List of available measurement ranges as returned by :py:func:`connect`.
        self.wavelength = 0
        self.MM_Wavelengths = ()    #: List of available wavelength settings as returned by :py:func:`connect`.
        self.measurement_running = False
        
        # connect to activeX component
        print("Try to connect to ActiveX component..")
        self.USBI_com = win32com.client.Dispatch("OphirLMMeasurement.CoLMMeasurement")      
        print("..success")
        print("COM Version:", self.USBI_com.GetVersion())
        
    def __del__(self):
        """Destructor.
        
        Automatically closes any open connection before termination.
        """
        # clean up
        self.USBI_com.CloseAll()
    
    def scanUSBI(self):
        """Scan for supported / connected power meters.
        
        :returns: List of attached USBI devices.
        """
        return self.USBI_com.ScanUSB()
        
    def connect(self, devID = 0):
        """Establish a connection to an Ophir USB power meter.
        
        The function acquires a list of connected Ophir devices and connects to the one found at position *devID*. After connecting, the current settings are loaded and a list of permitted values for the measurement parameters is generated. 
        
        :param int devID: Index of device to connect to. In order to connect to the first one, set this to 0 (default).
        :returns: None
        :raises IndexError: If devID does not point to a valid Ophir device.
        """
        # iterate all connected USB sensors
        devices = self.USBI_com.ScanUSB()
        print("Found", len(devices), "USB Devices..")
        
        # connect to device devID
        self.USBI_handle = self.USBI_com.OpenUSBDevice(devices[devID])
        print("Connected to Sensor", self.USBI_com.GetSensorInfo(self.USBI_handle, self.USBI_channel))
        
        # read configuration
        try:
            self.measurement_mode, self.MM_Modes = self.USBI_com.GetMeasurementMode(self.USBI_handle, self.USBI_channel)
            self.pulse_length, self.MM_PulseLengths = self.USBI_com.GetRanges(self.USBI_handle, self.USBI_channel)
            self.range, self.MM_Ranges = self.USBI_com.GetRanges(self.USBI_handle, self.USBI_channel)
            self.wavelength, self.MM_Wavelengths = self.USBI_com.GetWavelengths(self.USBI_handle, self.USBI_channel)
        except:
            pass
        
    def disconnect(self):
        """Close connection to current device.
        """
        if(self.USBI_handle != 0):
            self.USBI_com.Close(self.USBI_handle)
            self.USBI_handle = 0
    
    def reset(self):
        """Reset device.
        
        .. important:: The device must not be in streaming mode!
        """        
        if(self.USBI_handle != 0):
            self.USBI_com.ResetDevice(self.USBI_handle)
            
    # get / set methods
    def get_measurement_mode(self):
        """Returns the current measurement mode index.
        """
        return self.measurement_mode
        
    def set_measurement_mode(self, newmode):
        """Set new measurement mode.
        
        :param int newmode: Index of new measurement mode (0 to # of modes).
        :returns: None.
        """
        if(self.USBI_handle == 0):
            return
        if(newmode < 0 or newmode >= len(self.MM_Modes)):
            return
        self.measurement_mode = newmode
        self.USBI_com.SetMeasurementMode(self.USBI_handle, self.USBI_channel, newmode)
        
    def get_wavelength(self):
        """Returns index of current wavelength setting.
        """
        return self.wavelength
        
    def set_wavelength(self, newwl):
        """Set new wavelength setting.

        :param int newwl: Index of new wavelength setting (0 to # of wavelengths).
        :returns: None.
        """
        if(self.USBI_handle == 0):
            return
        if(newwl < 0 or newwl >= len(self.MM_Wavelengths)):
            return
        self.wavelength = newwl
        self.USBI_com.SetWavelength(self.USBI_handle, self.USBI_channel, newwl)
        
    def get_range(self):
        """Returns current power/energy range index.
        """
        return self.range
        
    def set_range(self, newrange):
        """Set new power/energy range setting.

        :param int newrange: Index of new range setting (0 to # of ranges).
        :returns: None.
        """
        if(self.USBI_handle == 0):
            return
        if(newrange < 0 or newrange >= len(self.MM_Ranges)):
            return
        self.range = newrange
        self.USBI_com.SetRange(self.USBI_handle, self.USBI_channel, newrange)
        
    def get_pulse_length(self):
        """Returns current pulse length setting index.
        """
        return self.pulse_length
        
    def set_pulse_length(self, newlength):
        """Set new pulse length setting.

        :param int newlength: Index of new pulse length setting (0 to # of pulse lengths).
        :returns: None.
        """
        if(self.USBI_handle == 0):
            return
        if(newlength < 0 or newlength >= len(self.MM_PulseLengths)):
            return
        self.pulse_length = newlength
        self.USBI_com.SetPulseLength(self.USBI_handle, self.USBI_channel, newlength)
        
    # measurement functions
    def set_turbo_mode(self, freq):
        """Set power meter to turbo mode. Only available if turbo mode is supported by hardware.
        
        :param float freq: Frequency for acquisition in Hz.
        :returns: None.
        """       
        if(self.USBI_handle == 0):
            return
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 2, 0)
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 1, freq)
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 0, 1)
    
    def set_immediate_mode(self):
        """Set power meter to immediate readout.
        """
        if(self.USBI_handle == 0):
            return
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 0, 0)
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 2, 1)
        
    def set_default_mode(self):
        """Set power meter to default readout mode.
        """
        if(self.USBI_handle == 0):
            return
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 0, 0)
        self.USBI_com.ConfigureStreamMode(self.USBI_handle, self.USBI_channel, 2, 0)
    
    def ask(self, cmd):
        """Function for direct legacy access to the Ophir power meter used for querying some value.
        
        For a list of commands see the OphirUSBI ActiveX manual.
        
        If reading of query result fails, an error message is printed to stdout.
            
        :param str cmd: Query command.
        :returns: Query result (*str*).
        """        
        if(self.USBI_handle == 0):
            return
        res = ""
        self.USBI_com.Write(self.USBI_handle, cmd)
        sleep(0.01) # wait 10ms
        try:            
            res = self.USBI_com.Read(self.USBI_handle)
        except:
            print("ERROR: Cannot read response from OphirUSBI!")
        return res.strip()

    def write(self, cmd):
        """Function for direct legacy access to the Ophir power meter.
        
        For a list of commands see the OphirUSBI ActiveX manual.
        
        :param str cmd: Query command.
        :returns: None.
        """                
        if(self.USBI_handle == 0):
            return
        self.USBI_com.Write(self.USBI_handle, cmd)
        sleep(0.01) # wait 10ms
        
    def read_data(self, num_samples):
        """Read data synchronously from power meter using the selected readout mode. 
        
        This function blocks execution of the program until the desired number of samples has been read.
        
        :param int num_samples: Number of data points to be read from the sensor.
        :returns: - Time of acquisition (*sequence*).
                  - Actual power meter reading (*sequence*).
        """
        if(self.USBI_handle == 0):
            return []
        datax = []
        datay = []      
        # poll data as long as n < num_samples
        time0 = time()
        ready = False
        mtime = 0       
        while(len(datax) < num_samples):
            data = ""
            ready = self.ask("EF")
            if ready == "*1":
                mtime = time() * 500
                if(self.measurement_mode == 0): # power
                    data = self.ask("SP")   
                    mtime += time() * 500
                else:   # energy
                    data = self.ask("SE")
                    mtime += time() * 500
                self.device_ready = True
                    
            if(data != ""): 
                time0 = time()      # reset timeout counter
                datax.append(mtime)
                datay.append(float(data[1:]))
            
            if time() - time0 > self.timeout:
                self.device_ready = False
                return []
            
        return [datax, datay]
    
    def start(self):
        """Starts an asynchronous acquisition of data from power meter using the selected measurement mode and returns immediately.
        
        Use :py:func:`stop` to stop acquisition and :py:func:`get_data` to read the data.
        """
        if(self.USBI_handle == 0):
            return
        if(self.measurement_running):
            return
        # start data capture
        self.USBI_com.StartStream(self.USBI_handle, self.USBI_channel)
        # set status variable
        self.measurement_running = True
        
    def stop(self):
        """Stops an asynchronous acquisition of data from power meter that has been started with :py:func:`start`.
        
        Use :py:func:`get_data` to read the data.
        """
        if(self.USBI_handle == 0):
            return
        # stop data capture
        self.USBI_com.StopStream(self.USBI_handle, self.USBI_channel)
        # set status variable
        self.measurement_running = False
    
    def get_data(self):
        """Read the result from an asynchronous data acquisition :py:func:`start`.
        
        .. warning:: Be careful with this function as it blocks the application if no data can be read from the sensor.
        
        :returns: - Time of acquisition (*sequence*).
                  - Actual power meter reading (*sequence*).
                  - Status of each sample (*sequence*).
        """
        if(self.USBI_handle == 0):
            return      
        if(self.device_ready == False):         
            self.read_data(1)                   # try to read data.. -> this would reset the device_ready flag
        if(self.device_ready == False):         # print an error if the device is still not operational
            print("ERROR: OphirUSBI Device not ready! Check range settings!")
            return []
        data = self.USBI_com.GetData(self.USBI_handle, self.USBI_channel)
        return [list(data[1]), list(data[0]), list(data[2])]
    