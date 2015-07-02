""" 
.. module: picam
   :platform: Windows
.. moduleauthor:: Daniel Dietze <daniel.dietze@berkeley.edu> 


..
   This file is part of the picam python module.

   The picam python module is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   The picam python module is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with the picam python module. If not, see <http://www.gnu.org/licenses/>.

   Copyright 2015 Daniel Dietze <daniel.dietze@berkeley.edu>.   
""" 
import os
import ctypes
import numpy as np
import threading
from picam_types import *

# ##########################################################################################################
# helper functions
def ptr(x):
    return ctypes.pointer(x)

# ##########################################################################################################
# Camera Class
class picam():
    """Main class that handles all connectivity with library and cameras.
    """
    # +++++++++++ CONSTRUCTION / DESTRUCTION ++++++++++++++++++++++++++++++++++++++++++++
    def __init__(self):
        """Constructor. 
        
        Calling this constructor does not connect with the library.
        """
        # empty handle
        self.cam = None
        self.camIDs = None
        self.roisPtr = []
        self.pulsePtr = []
        self.modPtr = []
        self.acqThread = None
        
    # load picam.dll and initialize library
    def loadLibrary(self, pathToLib = ""):
        """Load *picam.dll* and initialize library.
        
        :param str pathToLib: Path to the dynamic link library (optional). If empty, the library is loaded using the path given by the environment variabel *PicamRoot*, which is normally created by the PICam SDK installer.
        :returns: Prints the library version to stdout.
        """
        if pathToLib == "":
            pathToLib = os.path.join(os.environ["PicamRoot"], "Runtime")
        pathToLib = os.path.join(pathToLib, "Picam.dll")
        self.lib = ctypes.cdll.LoadLibrary(pathToLib)
        
        isconnected = pibln()
        self.status( self.lib.Picam_IsLibraryInitialized(ptr(isconnected)) )
        if not isconnected.value:
            self.status( self.lib.Picam_InitializeLibrary() )
    
        print self.getLibraryVersion()
    
    # call this function to release any resources and free the library
    def unloadLibrary(self):
        """Call this function to release any resources and free the library.
        """
        # clean up all reserved memory that may be around
        for i in range(len(self.roisPtr)):
            self.status( self.lib.Picam_DestroyRois(self.roisPtr[i]) )
        for i in range(len(self.pulsePtr)):
            self.status( self.lib.Picam_DestroyPulses(self.pulsePtr[i]) )
        for i in range(len(self.modPtr)):
            self.status( self.lib.Picam_DestroyModulations(self.modPtr[i]) )
    
        # disconnect from camera
        self.disconnect()
        
        if isinstance(self.camIDs, list):
            for c in self.camIDs:
                self.status( self.lib.Picam_DisconnectDemoCamera( ptr(c) ))
        
        # free camID resources
        if self.camIDs != None and not isinstance(self.camIDs, list):
            self.status( self.lib.Picam_DestroyCameraIDs( self.camIDs ) )
            self.camIDs = None
        
        # unload the library
        self.status( self.lib.Picam_UninitializeLibrary() )
        print "Unloaded PICamSDK"
    
    # +++++++++++ CLASS FUNCTIONS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # get version information
    def getLibraryVersion(self):
        """Returns the PICam library version string.
        """
        major = piint()
        minor = piint()
        distr = piint()
        released = piint()
        self.status( self.lib.Picam_GetVersion(ptr(major), ptr(minor), ptr(distr), ptr(released)) )
        return "PICam Library Version %d.%d.%d.%d" %(major.value, minor.value, distr.value, released.value)
    
    # returns a list of camera IDs that are connected to the computer
    # if no physical camera is found, a demo camera is initialized - for debug only
    def getAvailableCameras(self):
        """Queries a list of IDs of cameras that are connected to the computer and prints some sensor information for each camera to stdout.
        
        If no physical camera is found, a demo camera is initialized - *for debug only*.
        """
        if self.camIDs != None and not isinstance(self.camIDs, list):
            self.status( self.lib.Picam_DestroyCameraIDs( self.camIDs ) )
            self.camIDs = None
        
        # get connected cameras
        self.camIDs = ptr(PicamCameraID())
        id_count = piint()
        self.status( self.lib.Picam_GetAvailableCameraIDs(ptr(self.camIDs), ptr(id_count)) )
        
        # if none are found, create a demo camera
        print "Available Cameras:"
        if id_count.value < 1:
            self.status( self.lib.Picam_DestroyCameraIDs( self.camIDs ) )
            
            model_array = ptr(piint())
            model_count = piint()
            self.status( self.lib.Picam_GetAvailableDemoCameraModels( ptr(model_array), ptr(model_count) ) )
            
            model_ID = PicamCameraID()
            serial = ctypes.c_char_p("Demo Cam 1")
            self.status( self.lib.Picam_ConnectDemoCamera( model_array[0], serial, ptr(model_ID) ))
            self.camIDs = [model_ID]
            
            self.status( self.lib.Picam_DestroyModels(model_array) )
            
            print '  Model is ', PicamModelLookup[model_ID.model]
            print '  Computer interface is ', PicamComputerInterfaceLookup[model_ID.computer_interface]
            print '  Sensor_name is ', model_ID.sensor_name
            print '  Serial number is', model_ID.serial_number
            print '\n'
        else:
            for i in range(id_count.value):
                print '  Model is ', PicamModelLookup[self.camIDs[i].model]
                print '  Computer interface is ', PicamComputerInterfaceLookup[self.camIDs[i].computer_interface]
                print '  Sensor_name is ', self.camIDs[i].sensor_name
                print '  Serial number is', self.camIDs[i].serial_number
                print '\n'
    
    # returns string associated with last error
    def getLastError(self):
        """Returns the identifier associated with the last error (*str*).
        """
        return PicamErrorLookup[self.err]
    
    def status(self, err):
        """Checks the return value of a picam function for any error code. If an error occurred, it prints the error message to stdout.
        
        :param int err: Error code returned by any picam function call.
        :returns: Error code (int) and if an error occurred, prints error message.
        """
        errstr = PicamErrorLookup[err]
        if errstr != "None":
            print "ERROR: ", errstr
        #   raise AssertionError(errstr)
        self.err = err
        return err

    # connect / disconnect camera
    # if no camera ID is given, connect to the first available camera
    # otherwise camID is an integer index into a list of valid camera IDs that 
    # has been retrieved by getAvailableCameras()
    def connect(self, camID = None):
        """ Connect to camera.
        
        :param int camID:  Number of camera to connect to (optional). It is an integer index into a list of valid camera IDs that has been retrieved by :py:func:`getAvailableCameras`. If camID is None, this functions connects to the first available camera (default).
        """
        if self.cam != None:
            self.disconnect()
        if camID == None:
            self.cam = pivoid()
            self.status( self.lib.Picam_OpenFirstCamera( ptr(self.cam) ) )
        else:
            self.cam = pivoid()
            self.status( self.lib.Picam_OpenCamera(ptr(self.camIDs[camID]), ctypes.addressof(self.cam)) )
        # invoke commit parameters to validate all parameters for acquisition
        self.sendConfiguration()
        
    def disconnect(self):
        """Disconnect current camera.
        """
        if self.cam != None:
            self.status( self.lib.Picam_CloseCamera( self.cam ) )
        self.cam = None
    
    def getCurrentCameraID(self):
        """Returns the current camera ID (:py:class:`PicamCameraID`).
        """
        id = PicamCameraID()
        self.status( self.lib.Picam_GetCameraID(self.cam, ptr(id)) )
        return id
    
    # prints a list of parameters that are available
    def printAvailableParameters(self):
        """Prints an overview over the parameters that are available for the current camera and their limits.
        """        
        parameter_array = ptr(piint())
        parameter_count = piint()
        self.lib.Picam_GetParameters(self.cam, ptr(parameter_array), ptr(parameter_count))
        
        for i in range(parameter_count.value):
        
            # read / write access
            access = piint()
            self.lib.Picam_GetParameterValueAccess(self.cam, parameter_array[i], ptr(access));
            readable = PicamValueAccessLookup[access.value]
            
            # constraints
            contype = piint()
            self.lib.Picam_GetParameterConstraintType(self.cam, parameter_array[i], ptr(contype))
            
            if PicamConstraintTypeLookup[contype.value] == "None":
                constraint = "ALL"
                
            elif PicamConstraintTypeLookup[contype.value] == "Range":
                
                c = ptr(PicamRangeConstraint())
                self.lib.Picam_GetParameterRangeConstraint(self.cam, parameter_array[i], PicamConstraintCategory['Capable'], ptr(c))
                
                constraint = "from %f to %f in steps of %f" % (c[0].minimum, c[0].maximum, c[0].increment)
                
                self.lib.Picam_DestroyRangeConstraints(c)
                
            elif PicamConstraintTypeLookup[contype.value] == "Collection":
                
                c = ptr(PicamCollectionConstraint())
                self.lib.Picam_GetParameterCollectionConstraint(self.cam, parameter_array[i], PicamConstraintCategory['Capable'], ptr(c))
                
                constraint = ""
                for j in range(c[0].values_count):
                    if constraint != "":
                        constraint += ", "
                    constraint += str(c[0].values_array[j])
                
                self.lib.Picam_DestroyCollectionConstraints(c)
                
            elif PicamConstraintTypeLookup[contype.value] == "Rois":
                constraint = "N.A."
            elif PicamConstraintTypeLookup[contype.value] == "Pulse":
                constraint = "N.A."
            elif PicamConstraintTypeLookup[contype.value] == "Modulations":
                constraint = "N.A."
            
            # print infos
            print PicamParameterLookup[parameter_array[i]]
            print " value access:", readable
            print " allowed values:", constraint
            print "\n"
        
        self.lib.Picam_DestroyParameters(parameter_array)
        
    # get / set parameters
    # name is a string specifying the parameter
    def getParameter(self, name):
        """Reads and returns the value of the parameter with given name. If there is no parameter of this name, the function returns None and prints a warning.
        
        :param str name: Name of the parameter exactly as stated in the PICam SDK manual.
        :returns: Value of this parameter with data type corresponding to the type of parameter. 
        """
        prm = PicamParameter[name]
        
        exists = pibln()
        self.lib.Picam_DoesParameterExist(self.cam, prm, ptr(exists))
        if exists.value == False:
            print "Ignoring parameter", name
            print "  Parameter does not exist for current camera!"
            return None
        
        # get type of parameter
        type = piint()
        self.lib.Picam_GetParameterValueType(self.cam, prm, ptr(type));
        
        if type.value not in PicamValueTypeLookup:
            print "Not a valid parameter type enumeration:", type.value
            print "Ignoring parameter", name
            return 0
        
        if PicamValueTypeLookup[type.value] in ["Integer", "Boolean", "Enumeration"]:
            val = piint()
            if self.lib.Picam_GetParameterIntegerValue( self.cam, prm, ptr(val) ) == 0:
                return val.value
        
        if PicamValueTypeLookup[type.value] == "LargeInteger":
            val = pi64s()
            if self.lib.Picam_GetParameterLargeIntegerValue( self.cam, prm, ptr(val) ) == 0:
                return val.value
            
        if PicamValueTypeLookup[type.value] == "FloatingPoint":
            val = piflt()
            if self.lib.Picam_GetParameterFloatingPointValue( self.cam, prm, ptr(val) ) == 0:
                return val.value
        
        if PicamValueTypeLookup[type.value] == "Rois":
            val = ptr(PicamRois())
            if self.lib.Picam_GetParameterRoisValue( self.cam, prm, ptr(val) ) == 0:
                self.roisPtr.append(val)
                return val.contents
        
        if PicamValueTypeLookup[type.value] == "Pulse":
            val = ptr(PicamPulse())
            if self.lib.Picam_GetParameterPulseValue( self.cam, prm, ptr(val) ) == 0:
                self.pulsePtr.append(val)
                return val.contents
        
        if PicamValueTypeLookup[type.value] == "Modulations":
            val = ptr(PicamModulations())
            if self.lib.Picam_GetParameterModulationsValue( self.cam, prm, ptr(val) ) == 0:
                self.modPtr.append(val)
                return val.contents
        
        return None
        
    def setParameter(self, name, value):
        """Set parameter. The value is automatically typecast to the correct data type corresponding to the type of parameter.
        
        :param str name: Name of the parameter exactly as stated in the PICam SDK manual.
        :param mixed value: New parameter value. If the parameter value cannot be changed, a warning is printed to stdout.
        
        .. note:: Setting a parameter with this function does not automatically change the configuration in the camera. In order to apply all changes, :py:func:`sendConfiguration` has to be called.
        """        
        prm = PicamParameter[name]
        
        exists = pibln()
        self.lib.Picam_DoesParameterExist(self.cam, prm, ptr(exists))
        if not exists:
            print "Ignoring parameter", name
            print "  Parameter does not exist for current camera!"
            return 
        
        access = piint()
        self.lib.Picam_GetParameterValueAccess(self.cam, prm, ptr(access))
        if PicamValueAccessLookup[access.value] not in ["ReadWrite", "ReadWriteTrivial"]:
            print "Ignoring parameter", name
            print "  Not allowed to overwrite parameter!"
            return 
        if PicamValueAccessLookup[access.value] == "ReadWriteTrivial":
            print "WARNING: Parameter", name, " allows only one value!"
        
        # get type of parameter
        type = piint()
        self.lib.Picam_GetParameterValueType(self.cam, prm, ptr(type));
        
        if type.value not in PicamValueTypeLookup:
            print "Ignoring parameter", name
            print "  Not a valid parameter type:", type.value
            return 
            
        if PicamValueTypeLookup[type.value] in ["Integer", "Boolean", "Enumeration"]:
            val = piint(value)
            self.status( self.lib.Picam_SetParameterIntegerValue(self.cam, prm, val) )
            
        if PicamValueTypeLookup[type.value] == "LargeInteger":
            val = pi64s(value)
            self.status( self.lib.Picam_SetParameterLargeIntegerValue(self.cam, prm, val) ) 
        
        if PicamValueTypeLookup[type.value] == "FloatingPoint":
            val = piflt(value)
            self.status( self.lib.Picam_SetParameterFloatingPointValue(self.cam, prm, val) )
        
        if PicamValueTypeLookup[type.value] == "Rois":
            self.status( self.lib.Picam_SetParameterRoisValue( self.cam, prm, ptr(value) ))
                
        if PicamValueTypeLookup[type.value] == "Pulse":
            self.status( self.lib.Picam_SetParameterPulseValue( self.cam, prm, ptr(value) ))
            
        if PicamValueTypeLookup[type.value] == "Modulations":
            self.status( self.lib.Picam_SetParameterModulationsValue( self.cam, prm, ptr(value) ))
        
        if self.err != PicamError["None"]:
            print "Ignoring parameter", name
            print "  Could not change parameter. Keeping previous value:", self.getParameter(name)
        
    # this function has to be called once all configurations
    # are done to apply settings to the camera
    def sendConfiguration(self):
        """This function has to be called once all configurations are done to apply settings to the camera.
        """
        failed = ptr(piint())
        failedCount = piint()
        
        self.status( self.lib.Picam_CommitParameters(self.cam, ptr(failed), ptr(failedCount) ))
        
        if failedCount.value > 0:
            for i in range(failedCount.value):
                print "Could not set parameter", PicamParameterLookup[failed[i]]
        self.status( self.lib.Picam_DestroyParameters(failed) )
    
        self.updateROIS()
    
    # utility function that extracts the number of pixel sizes of all ROIs - used internally, hide in docs
    def updateROIS(self):
        self.ROIS = []
        rois = self.getParameter("Rois")
        for i in range(rois.roi_count):
            w = int(np.ceil(float(rois.roi_array[i].width) / float(rois.roi_array[i].x_binning)))
            h = int(np.ceil(float(rois.roi_array[i].height) / float(rois.roi_array[i].y_binning)))
            self.ROIS.append([w, h])

    # set a single ROI
    def setROI(self, x0, w, xbin, y0, h, ybin):
        """Create a single region of interest.
        
        :param int x0: X-coordinate of upper left corner of ROI.
        :param int w: Width of ROI.
        :param int xbin: X-Binning, i.e. number of columns that are combined into one larger column (1 to w).
        :param int y0: Y-coordinate of upper left corner of ROI.
        :param int h: Height of ROI.
        :param int ybin: Y-Binning, i.e. number of rows that are combined into one larger row (1 to h).
        """
        r = PicamRoi(x0, w, xbin, y0, h, ybin)
        R = PicamRois( ptr(r), 1 )
        self.setParameter("Rois", R)
    
        self.updateROIS()
    
    # add a ROI
    def addROI(self, x0, w, xbin, y0, h, ybin):
        """Add a region-of-interest to the existing list of ROIs. 
        
        .. important:: The ROIs should not overlap!
        
        :param int x0: X-coordinate of upper left corner of ROI.
        :param int w: Width of ROI.
        :param int xbin: X-Binning, i.e. number of columns that are combined into one larger column (1 to w).
        :param int y0: Y-coordinate of upper left corner of ROI.
        :param int h: Height of ROI.
        :param int ybin: Y-Binning, i.e. number of rows that are combined into one larger row (1 to h).
        """
        # read existing rois
        R = self.getParameter("Rois")
        r0 = (PicamRoi * (R.roi_count + 1))()
        for i in range(R.roi_count):
            r0[i] = R.roi_array[i]
        
        # add new roi
        r0[-1] = PicamRoi(x0, w, xbin, y0, h, ybin)
    
        # write back to camera
        R1 = PicamRois( ptr(r0[0]), len(r0) )
        self.setParameter("Rois", R1)
    
        self.updateROIS()
    
    # acquisition functions
    # readNFrames waits till all frames have been collected (using Picam_Acquire)
    # if you want non-blocking behavior, use start/stopAcquisition instead
    
    # N = number of frames
    # timeout = max wait time between frames in ms
    def readNFrames(self, N = 1, timeout = 1000):
        """This function acquires N frames using Picam_Acquire. It waits till all frames have been collected before it returns. If you want non-blocking behavior, use :py:func:`startAcquisition` and :py:func:`stopAcquisition` instead.
        
        :param int N: Number of frames to collect (>= 1). This number is essentially limited by the available memory.
        :param float timeout: Maximum wait time between frames in milliseconds. This parameter is important when using external triggering.
        :returns: Numpy array of acquired frames.
        """
        available = PicamAvailableData()
        errors = piint()
        
        # start acquisition
        self.status( self.lib.Picam_Acquire( self.cam, pi64s(N), piint(timeout), ptr(available), ptr(errors) ))
        
        if PicamErrorLookup[self.err] == "None" and PicamAcquisitionErrorsMaskLookup[errors.value] == "None":
            
            # return data as numpy array
            return self.getBuffer(available.initial_readout, available.readout_count, N)
        
        else:
            print "Acquisition ERROR:", PicamAcquisitionErrorsMaskLookup[errors.value]
            
            return []
    
    # this is a helper function that converts a readout buffer into a sequence of numpy arrays
    # it reads all available data at once into a numpy buffer and reformats data to fit to the output mask
    # returns data as floating point
    def getBuffer(self, adress, size, maxN = -1):
        
        # get number of bytes contained in a single readout and a single frame
        # parameters are bytes, one pixel in resulting array is 2 bytes
        readoutstride = self.getParameter("ReadoutStride") / 2
        framestride = self.getParameter("FrameStride") / 2
        frames = self.getParameter("FramesPerReadout")

        # create a pointer to data
        dataArrayType = pi16u * readoutstride * size
        dataArrayPointerType = ctypes.POINTER(dataArrayType)
        dataPointer = ctypes.cast(adress, dataArrayPointerType)
        
        # create a numpy array from the buffer
        data = np.frombuffer(dataPointer.contents, dtype='uint16').astype(np.float)
        
        # the returned array shall be a sequence of ROIs:
        # [ ROI1, ROI2, ROI3, .. ]
        # where ROI is sequence of frames
        # [ frame 1, frame 2, ... ]
        # and each frame is 2d data
        # [ [ 11, 12, 13, ...], [ 21, 22, 23, ..], ... ]
        
        # create empty output array
        out = [[] for i in range(len(self.ROIS))]
        
        # limit number of frames
        if maxN != -1:
            size = min(size, maxN)
        
        # iterate through data and append frames to array
        for i in range(size):
            for j in range(frames):
                ofs = 0
                for k in range(len(self.ROIS)):
                    try:
                        tmp = ((data[i*readoutstride:(i+1)*readoutstride])[j*framestride:(j+1)*framestride])[ofs:ofs + self.ROIS[k][0] * self.ROIS[k][1]]
                        out[k].append(tmp.reshape((self.ROIS[k][0], self.ROIS[k][1])))
                        ofs =+ self.ROIS[k][0] * self.ROIS[k][1]
                    except:
                        print "readoutstride =" , readoutstride
                        print "framestride =", framestride
                        print "no of frames =", frames
                        print "buffer size =", tmp.shape
                        print "roi size =", self.ROIS[k][0], "x", self.ROIS[k][1]
                        return []
                        
        return out

    def startAcquisition(self, N = 1):
        """Start asynchronous acquisition of data from the camera. Returns immediately after starting the acquisition.
        
        :param int N: Number of frames that should be recorded.
        """
        # send number of frames to camera
        self.setParameter("ReadoutCount", N)
        self.sendConfiguration()
        
        self.acqThread = picamThread(self.lib, self.cam, self.getParameter("ReadoutStride") / 2)
        self.acqThread.start()
    
    def acquisitionRunning(self):
        """Returns status of acquisition (True / False).
        """
        # running = pibln()
        # self.status( self.lib.Picam_IsAcquisitionRunning(self.cam, ptr(running)) )
        # return running.value
        if self.acqThread != None and self.acqThread.is_alive():
            return True
        return False
    
    def stopAcquisition(self):
        """Finishes an asynchronous data acquisition. If the acquisition is still running, it will be terminated.
        
        :returns: Numpy array of recorded frames.
        """        
        if self.acquisitionRunning():
            self.acqThread.stop()
            self.acqThread.join()
        
        # get number of bytes contained in a single readout and a single frame
        # parameters are bytes, on pixel in resulting array is 2 bytes
        readoutstride = self.getParameter("ReadoutStride") / 2
        framestride = self.getParameter("FrameStride") / 2
        frames = self.getParameter("FramesPerReadout")
        
        size = self.acqThread.counts
        data = self.acqThread.data
        
        # create empty output array
        out = [[] for i in range(len(self.ROIS))]
        
        # iterate through data and append frames to array
        for i in range(size):
            for j in range(frames):
                ofs = 0
                for k in range(len(self.ROIS)):
                    try:
                        tmp = ((data[i*readoutstride:(i+1)*readoutstride])[j*framestride:(j+1)*framestride])[ofs:ofs + self.ROIS[k][0] * self.ROIS[k][1]]
                        out[k].append(tmp.reshape((self.ROIS[k][0], self.ROIS[k][1])))
                        ofs =+ self.ROIS[k][0] * self.ROIS[k][1]
                    except:
                        print "readoutstride =" , readoutstride
                        print "framestride =", framestride
                        print "no of frames =", frames
                        print "buffer size =", tmp.shape
                        print "roi size =", self.ROIS[k][0], "x", self.ROIS[k][1]
        
        return out

    
# ################################################################################
# helper class for data acquisition that allows user interruption
class picamThread(threading.Thread):
    def __init__(self, lib, cam, readoutstride, **argv):
        threading.Thread.__init__(self)
        
        # image data
        self.data = np.array([])
        self.counts = 0
        self.readoutstride = readoutstride
        
        # camera handle
        self.cam = cam
        
        # library handle
        self.lib = lib
        
        # user stop event, handles also sleep-functionality
        self.canQuit = threading.Event()
        self.canQuit.clear()
        
    # the main GUI calls this function to terminate the thread
    def stop(self):
        self.canQuit.set()
        
    # this is the actual scan routine
    def run(self):
        available = PicamAvailableData()
        status = PicamAcquisitionStatus()
        timeout = piint(-1)
        
        # start acquisition
        self.lib.Picam_StartAcquisition(self.cam) 
        
        # enter main loop
        while(self.canQuit.isSet() == 0):
            
            # check for data
            self.lib.Picam_WaitForAcquisitionUpdate(self.cam, timeout, ptr(available), ptr(status))
            
            # if data available, read it to data buffer
            if available.readout_count > 0:
                size = available.readout_count
                self.counts += size
                
                # create a pointer to data
                dataArrayType = pi16u * self.readoutstride * size
                dataArrayPointerType = ctypes.POINTER(dataArrayType)
                dataPointer = ctypes.cast(available.initial_readout, dataArrayPointerType)
            
                # create a numpy array from the buffer
                tmp = np.frombuffer(dataPointer.contents, dtype='uint16').astype(np.float)
        
                self.data = np.append(self.data, tmp)
            
            # check whether we are done
            if status.running == False:
                break
        
        # stop running acquisition if canQuit is set
        if self.canQuit.isSet():
            self.lib.Picam_StopAcquisition(self.cam) 

if __name__ == '__main__':

    import sys
    import time
    
    cam = picam()
    cam.loadLibrary()
    cam.getAvailableCameras()
    cam.connect()
    
    # cool down CCD
#   cam.setParameter("SensorTemperatureSetPoint", -75)
    
    # shortest expoure
    cam.setParameter("ExposureTime", 0)
    
    # readout mode
    cam.setParameter("ReadoutControlMode", PicamReadoutControlMode["FullFrame"])
        
    # custom chip settings
    cam.setROI(0, 1339, 1, 0, 99, 100)
    cam.setParameter("ActiveWidth", 1340) 
    cam.setParameter("ActiveHeight", 100)
    cam.setParameter("ActiveLeftMargin", 0) 
    cam.setParameter("ActiveRightMargin", 0)
    cam.setParameter("ActiveTopMargin", 8) 
    cam.setParameter("ActiveBottomMargin", 8)
    cam.setParameter("VerticalShiftRate", 3.2)  # select fastest
    
    # set logic out to not ready
    cam.setParameter("OutputSignal", PicamOutputSignal["Busy"])
    
    # shutter delays; open before trigger corresponds to shutter opening pre delay
    cam.setParameter("ShutterTimingMode", PicamShutterTimingMode["Normal"]) # OpenBeforeTrigger, Normal
    cam.setParameter("ShutterClosingDelay", 0)
    
    # sensor cleaning
    cam.setParameter("CleanSectionFinalHeightCount", 1)
    cam.setParameter("CleanSectionFinalHeight", 100)
    cam.setParameter("CleanSerialRegister", False)
    cam.setParameter("CleanCycleCount", 1)
    cam.setParameter("CleanCycleHeight", 100)
    cam.setParameter("CleanUntilTrigger", True)
    
    # sensor gain settings
    # according to manual, Pixis supports 100kHz and 2MHz; select fastest
    cam.setParameter("AdcSpeed", 2.0)
    cam.setParameter("AdcAnalogGain", PicamAdcAnalogGain["Low"])
    cam.setParameter("AdcQuality", PicamAdcQuality["HighCapacity"])
    
    # trigger and timing settings
    cam.setParameter("TriggerDetermination", PicamTriggerDetermination["PositivePolarity"])
    cam.setParameter("TriggerResponse", PicamTriggerResponse["ReadoutPerTrigger"])
    
    # send configuration
    cam.sendConfiguration()
    
    # get readout speed
    print "Estimated readout time = %f ms" % cam.getParameter("ReadoutTimeCalculation")
    
    cam.disconnect()
    cam.unloadLibrary()
    