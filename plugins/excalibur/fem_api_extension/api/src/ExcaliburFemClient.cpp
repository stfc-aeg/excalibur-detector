/*
 * ExcaliburFemClient.cpp - EXCALIBUR FEM client class implementation
 *
 *  Created on: 7 Dec 2011
 *      Author: Tim Nicholls, STFC Application Engineering Group
 */

#include "ExcaliburFemClient.h"
#include "ExcaliburFrontEndDevices.h"
#include "asicControlParameters.h"
#include "mpx3Parameters.h"
#include "time.h"
#include <map>
#include <iostream>
#include <iomanip>
#include <sstream>

typedef void (ExcaliburFemClient::*ExcaliburScanFunc)(void);

ExcaliburFemClient::ExcaliburFemClient(void* aCtlHandle, const CtlCallbacks* aCallbacks,
		const CtlConfig* aConfig, unsigned int aTimeoutInMsecs) :
	FemClient(aConfig->femAddress, aConfig->femPort, aTimeoutInMsecs),
	mFemId(aConfig->femNumber),
	mMpx3GlobalTestPulseEnable(false),
	mMpx3TestPulseCount(4000),
	mDataReceiverEnable(true),
	mFemDataHostPort(kHostDataPort),
	mCtlHandle(aCtlHandle),
	mCallbacks(aCallbacks),
	mConfig(aConfig),
	mAsicDataReorderMode(reorderedDataMode),
	mNumSubFrames(2),
	mTriggerMode(excaliburTriggerModeInternal),
	mTriggerPolarity(excaliburTriggerPolarityActiveHigh),
	mBurstModeSubmitPeriod(0),
	mLfsrBypassEnable(false),
	mEnableDeferredBufferRelease(false),
	mDacScanDac(0),
	mDacScanStart(0),
	mDacScanStop(0),
	mDacScanStep(0)
{

	// Initialize MPX3 DAC settings, values and pixel config caches to zero
	for (unsigned int iChip = 0; iChip < kNumAsicsPerFem; iChip++)
	{
		for (unsigned int iDac = 0; iDac < numExcaliburDacs; iDac++)
		{
			mMpx3DacCache[iChip][iDac] = 0;
		}

		for (unsigned int iConfig= 0; iConfig < numPixelConfigs; iConfig++) {
			for (unsigned int iPixel = 0; iPixel < kNumPixelsPerAsic; iPixel++)
			{
				mMpx3PixelConfigCache[iChip][iConfig][iPixel]       = 0;
			}
		}

		// Initialise default values for some standard parameters used in all
		// OMR transactions
		mMpx3OmrParams[iChip].readWriteMode         = sequentialReadWriteMode;
//		mMpx3OmrParams[iChip].polarity              = electronPolarity;
		mMpx3OmrParams[iChip].polarity              = holePolarity;
		mMpx3OmrParams[iChip].readoutWidth          = readoutWidth8;
		mMpx3OmrParams[iChip].discCsmSpm            = discCsmSpmDiscL;
		mMpx3OmrParams[iChip].testPulseEnable       = 0;
		mMpx3OmrParams[iChip].counterDepth          = counterDepth12;
		mMpx3OmrParams[iChip].columnBlock           = 0;
		mMpx3OmrParams[iChip].columnBlockSelect     = 0;
		mMpx3OmrParams[iChip].rowBlock              = 0;
		mMpx3OmrParams[iChip].rowBlockSelect        = 0;
		mMpx3OmrParams[iChip].equalizationMode      = equalizationModeDisabled;
		mMpx3OmrParams[iChip].colourMode            = monochromeMode;
		mMpx3OmrParams[iChip].csmSpmMode            = csmSpmModeSpm;
		mMpx3OmrParams[iChip].infoHeaderEnable      = 0;
		mMpx3OmrParams[iChip].fuseSel               = 0;
		mMpx3OmrParams[iChip].fusePulseWidth        = 0;
		mMpx3OmrParams[iChip].gainMode				= gainModeSuperLow;
		mMpx3OmrParams[iChip].dacSense              = 0;
		mMpx3OmrParams[iChip].dacExternal           = 0;
		mMpx3OmrParams[iChip].externalBandGapSelect = 0;

		// Initialise chip column test pulse enables
		for (unsigned int iCol = 0; iCol < kNumColsPerAsic; iCol++)
		{
			mMpx3ColumnTestPulseEnable[iChip][iCol] = 0;
		}

		// Initialise chip enable flag
		mMpx3Enable[iChip] = true;

	}

	// Reset the ASIC control f/w block and ASICS
//	this->asicControlReset();
//	this->asicControlAsicReset();

	// Initialise front-end DACs
	//this->frontEndDacInitialise();

	// Build callback bundle to be registered with the data receiver
	mCallbackBundle.allocate = boost::bind(&ExcaliburFemClient::allocateCallback, this);
	mCallbackBundle.free     = boost::bind(&ExcaliburFemClient::freeCallback, this, _1);
	mCallbackBundle.receive  = boost::bind(&ExcaliburFemClient::receiveCallback, this, _1, _2);
	mCallbackBundle.signal   = boost::bind(&ExcaliburFemClient::signalCallback, this, _1);


	// Resolve data connection source and destination IP addresses from config
	const char* hostIpAddress = aConfig->dataAddress;
	char* fpgaIpAddress = this->getFpgaIpAddressFromHost(hostIpAddress);
	const char* fpgaMacAddress = "62:00:00:00:00:01";
	u32 hostPort = kHostDataPort;
	u32 fpgaPort = 8;
	std::cout << "Configuring 10GigE data interface: host IP: " << hostIpAddress << " port: " << hostPort
			  << " FEM data IP: " << fpgaIpAddress << " port: " << fpgaPort << " MAC: " << fpgaMacAddress << std::endl;

	// Initialise 10GigE UDP interface in FEM
	u32 rc = this->configUDP((char *)fpgaMacAddress, fpgaIpAddress, fpgaPort, (char *)hostIpAddress, hostPort);
	if (rc != 0)
	{
		throw FemClientException((FemClientErrorCode)excaliburFemClientUdpSetupFailed, "Failed to set up FEM UDP firmware block");
	}

//	try
//	{
//		mFemDataReceiver = new FemDataReceiver(mFemDataHostPort);
//	}
//	catch (boost::system::system_error &e)
//	{
//		std::ostringstream msg;
//		msg << "Failed to create FEM data receiver: " << e.what();
//		throw FemClientException((FemClientErrorCode)excaliburFemClientDataReceviverSetupFailed, msg.str());
//	}

	// Check DMA engine acquisition state and reset to IDLE if in a different state
	FemAcquireStatus acqStatus = this->acquireStatus();
	if (acqStatus.state != acquireIdle)
	{
		std::cout << "Acquisition state at startup is " << acqStatus.state << " sending stop to reset" << std::endl;
		this->acquireStop();
	}
	else
	{
		std::cout << "Acquisition state is IDLE at startup" << std::endl;
	}
}


ExcaliburFemClient::~ExcaliburFemClient() {

	// Delete the data receiver object if it was created
	if (mFemDataReceiver)
	{
		delete (mFemDataReceiver);
	}

}

int ExcaliburFemClient::get_id(void) {

	return mFemId;
}

BufferInfo ExcaliburFemClient::allocateCallback(void)
{

	BufferInfo buffer;
	CtlFrame* frame;

	// If the frame queue is empty (i.e. no pre-allocated frame buffers), request
	// a frame via the callback, otherwise use the front-most frame in the queue

	if (mFrameQueue.empty())
	{
		frame = mCallbacks->ctlAllocate(mCtlHandle);
		mFrameQueue.push_back(frame);
	}
	else
	{
		frame = mFrameQueue.front();
	}

	// TODO handle frame being NULL here

	// Map the frame information into the buffer to return
	buffer.addr    = (u8*)(frame->buffer);
	buffer.length = (frame->bufferLength);

	return buffer;

}


void ExcaliburFemClient::freeCallback(int aVal)
{

	mCallbacks->ctlFree(mCtlHandle, 0);

}


void ExcaliburFemClient::receiveCallback(int aFrameCounter, time_t aRecvTime)
{

	// Get the first frame on our queue
	CtlFrame* frame = mFrameQueue.front();

	// Fill fields into frame metadata
	frame->frameCounter = aFrameCounter;
	frame->timeStamp    = aRecvTime;

	// If deferred buffer release is enabled, queue the completed
	// frame on the release queue, otherwise call the receive callback
	// to release the frame
	if (mEnableDeferredBufferRelease)
	{
		mReleaseQueue.push_back(frame);
	}
	else
	{
		mCallbacks->ctlReceive(mCtlHandle,frame);
	}

	// Pop the frame off the queue
	mFrameQueue.pop_front();

}


void ExcaliburFemClient::signalCallback(int aSignal)
{

	int theSignal;

	switch (aSignal)
	{

	case FemDataReceiverSignal::femAcquisitionComplete:

		theSignal = FEM_OP_ACQUISITIONCOMPLETE;
		std::cout << "Got acquisition complete signal" << std::endl;
		//this->acquireStop();
		//this->stopAcquisition();

		// If deferred buffer release is enabled, drain the release queue
		// out through the receive callback at the requested rate
		if (mEnableDeferredBufferRelease)
		{
			this->releaseAllFrames();
		}
		break;

	case FemDataReceiverSignal::femAcquisitionCorruptImage:
		theSignal = FEM_OP_CORRUPTIMAGE;
		std::cout << "Got corrupt image signal" << std::endl;
		break;

	default:
		theSignal = aSignal;
		break;

	}

	mCallbacks->ctlSignal(mCtlHandle, theSignal);

}

void ExcaliburFemClient::preallocateFrames(unsigned int aNumFrames)
{
	for (unsigned int i = 0; i < aNumFrames; i++)
	{
		CtlFrame* frame = mCallbacks->ctlAllocate(mCtlHandle);
		if (frame != NULL)
		{
			mFrameQueue.push_back(frame);
		}
		else
		{
			throw FemClientException((FemClientErrorCode)excaliburFemClientBufferAllocateFailed, "Buffer allocation callback failed");
		}
	}
	std::cout << "Preallocate complete - frame queue size is now " << mFrameQueue.size() << std::endl;
}

void ExcaliburFemClient::releaseAllFrames(void)
{
	int numFramesToRelease = mReleaseQueue.size();
	std::cout << "Deferred buffer release - draining release queue of "
			  << numFramesToRelease << " frames" << std::endl;

	// Build a timespec for the release period
	time_t releaseSecs = (time_t)((int)mBurstModeSubmitPeriod);
	long   releaseNsecs = (long)((mBurstModeSubmitPeriod - releaseSecs)*1E9);
	struct timespec releasePeriod = {releaseSecs, releaseNsecs};

	struct timespec startTime, endTime;
	clock_gettime(CLOCK_REALTIME, &startTime);

	while (mReleaseQueue.size() > 0)
	{
		CtlFrame* frame = mReleaseQueue.front();
		mCallbacks->ctlReceive(mCtlHandle, frame);
		mReleaseQueue.pop_front();
		if (mBurstModeSubmitPeriod > 0.0)
		{
			nanosleep((const struct timespec *)&releasePeriod, NULL);
		}
	}

	clock_gettime(CLOCK_REALTIME, &endTime);
	double startSecs = startTime.tv_sec  + ((double)startTime.tv_nsec / 1.0E9);
	double endSecs   = endTime.tv_sec  + ((double)endTime.tv_nsec / 1.0E9);
	double elapsedSecs = endSecs - startSecs;
	double elapsedRate = (double)numFramesToRelease / elapsedSecs;

	std::cout << "Release completed: " << numFramesToRelease << " frames released in "
			  << elapsedSecs << " secs, rate: " << elapsedRate << " Hz"
			  << std::endl;

}

void ExcaliburFemClient::freeAllFrames(void)
{

	while (mFrameQueue.size() > 0)
	{
		CtlFrame* frame = mFrameQueue.front();
		mCallbacks->ctlFree(mCtlHandle, frame);
		mFrameQueue.pop_front();
	}

}
void ExcaliburFemClient::command(unsigned int aCommand)
{

	unsigned int theCommand = 0;
	switch (aCommand)
	{
	case FEM_OP_STARTACQUISITION:
		this->startAcquisition();
		//this->toyAcquisition();
		break;

	case FEM_OP_STOPACQUISITION:
		this->stopAcquisition();
		break;

	default:
		theCommand = aCommand;
		FemClient::command(theCommand);
		break;
	}


}

void ExcaliburFemClient::toyAcquisition(void)
{
	std::cout << "Running toy acquisition loop for numFrames=" << mNumFrames << std::endl;
	for (unsigned int iBuffer = 0; iBuffer < mNumFrames; iBuffer++)
	{
		BufferInfo aBuffer = this->allocateCallback();
		this->receiveCallback(iBuffer, (time_t)1234);
	}
	this->signalCallback(FemDataReceiverSignal::femAcquisitionComplete);
	std::cout << "Ending toy acq loop" << std::endl;

}


void ExcaliburFemClient::startAcquisition(void)
{

	struct timespec startTime, endTime;

	clock_gettime(CLOCK_REALTIME, &startTime);

	// Create a data receiver object if enabled
	if (mDataReceiverEnable)
	{
		try
		{
			mFemDataReceiver = new FemDataReceiver(mFemDataHostPort);
		}
		catch (boost::system::system_error &e)
		{
			std::ostringstream msg;
			msg << "Failed to create FEM data receiver: " << e.what();
			throw FemClientException((FemClientErrorCode)excaliburFemClientDataReceviverSetupFailed, msg.str());
		}
	}


	// Default values for various acquisition parameters
	u32 acqMode, numAcq, bdCoalesce = 0;
	unsigned int numRxFrames = mNumFrames; // Default data receiver to receive specified number of frames
	bool bufferPreAllocate = false;
	bool clientAcquisitionControl = true;
	bool enableFrameCounterCheck = true;
	ExcaliburScanFunc theScanFunc = NULL;
	unsigned int executeCmd = asicPixelMatrixRead;
	mpx3CounterSelect counterSelect = mMpx3CounterSelect;
	bool doMatrixClearFirst = true;

	// Select various parameters based on operation mode
	switch (mOperationMode)
	{

	case excaliburOperationModeNormal:
		acqMode = ACQ_MODE_NORMAL;
		numAcq = 0; // Let the acquire config command configure all buffers in this mode
		bdCoalesce = 1;
		mEnableDeferredBufferRelease = false;
		break;

	case excaliburOperationModeBurst:
		acqMode = ACQ_MODE_BURST;
		numAcq = mNumFrames;
		bdCoalesce = 1;
		mEnableDeferredBufferRelease = true;
		enableFrameCounterCheck = false;
		bufferPreAllocate = true;
		break;

	case excaliburOperationModeDacScan:
		acqMode = ACQ_MODE_NORMAL;
		numAcq = 0;
		bdCoalesce = 1;
		mEnableDeferredBufferRelease = false;
		enableFrameCounterCheck = false;
		numRxFrames = this->dacScanNumSteps(); // Override number of frames to receive based on number of steps in scan
		clientAcquisitionControl = false;      // FEM will be in control of acquisition sequence for DAC scan
		theScanFunc = &ExcaliburFemClient::dacScanExecute;    // Set scan function pointer to DAC scan execute member function
		break;

	case excaliburOperationModeMatrixRead:
		acqMode = ACQ_MODE_NORMAL;
		numAcq = 0;
		bdCoalesce = 1;
		mEnableDeferredBufferRelease = false;
		enableFrameCounterCheck = false;
		numRxFrames = 1; // Override number of frames for a single read of the pixel matrix
		executeCmd = asicPixelConfigRead; // Override default execute command for this mode
		doMatrixClearFirst = false;
		break;

	case excaliburOperationModeHistogram:
		// Deliberate fall-thru as histogram mode not yet supported

	default:
		{
			std::ostringstream msg;
			msg << "Cannot start acquisition, illegal operation mode specified: " << mOperationMode;
			throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalOperationMode, msg.str());
		}
		break;
	}

	// Select LFSR decoding and data reordering modes based on defaults and counter depth

	asicLfsrDecodeMode lfsrMode;
	asicDataReorderMode reorderMode = mAsicDataReorderMode;
	if (mLfsrBypassEnable) {
		std::cout << "LFSR decoding bypass is enabled" << std::endl;
	}
	switch (mMpx3OmrParams[0].counterDepth)
	{
	case counterDepth1:
		lfsrMode = lfsr12Bypass;
		reorderMode = rawDataMode;
		enableFrameCounterCheck = false;
		break;
	case counterDepth6:
		lfsrMode = mLfsrBypassEnable ? lfsr6Bypass : lfsr6Enable;
		break;
	case counterDepth12:
	case counterDepth24:
//		reorderMode = rawDataMode;
//		enableFrameCounterCheck = false;
		lfsrMode = mLfsrBypassEnable ? lfsr12Bypass : lfsr12Enable;
		break;
	default:
		{
			std::ostringstream msg;
			msg << "Cannot start acquisition, illegal counter depth specified: " << mMpx3OmrParams[0].counterDepth;
			throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalCounterDepth, msg.str());
		}
		break;
	}


	// Execute a fast matrix clear if necessary for this mode
	if (doMatrixClearFirst)
	{
		std::cout << "Executing ASIC fast matrix clear" << std::endl;
		this->asicControlFastMatrixClear();
		usleep(10);
	}

	// Set up counter depth for ASIC control based on current OMR settings
	this->asicControlCounterDepthSet(mMpx3OmrParams[0].counterDepth);

	// Set LFSR decode mode
	this->asicControlLfsrDecodeModeSet(lfsrMode);

	// Set ASIC data reordering mode
	this->asicControlDataReorderModeSet(reorderMode);

	// Set up the readout length in clock cycles for the ASIC control block
	unsigned int readoutLengthCycles = this->asicReadoutLengthCycles();
	this->asicControlReadoutLengthSet(readoutLengthCycles);

	// Set up the acquisition DMA controller and arm it, based on operation mode
	unsigned int dmaSize = this->asicReadoutDmaSize();
	//std::cout << "Configuring DMA controller" << std::endl;
	this->acquireConfig(acqMode, dmaSize, 0, numAcq, bdCoalesce);
	//std::cout << "Starting DMA controller" << std::endl;
	this->acquireStart();
	//std::cout << "Done" <<std::endl;

	if (mDataReceiverEnable)
	{

		// Pre-allocate frame buffers for data receiver if necessary
		if (bufferPreAllocate)
		{
			this->preallocateFrames(numRxFrames);
		}

		// Register callbacks for data receiver
		mFemDataReceiver->registerCallbacks(&mCallbackBundle);

		// Set up the number of frames, acquisition period and time for the receiver thread
		mFemDataReceiver->setNumFrames(numRxFrames);
		mFemDataReceiver->setAcquisitionPeriod(mAcquisitionPeriodMs);
		mFemDataReceiver->setAcquisitionTime(mAcquisitionTimeMs);

		// Set up frame length and header sizes for the data receiver thread
		mFemDataReceiver->setFrameHeaderLength(8);
		mFemDataReceiver->setFrameHeaderPosition(headerAtStart);
		mFemDataReceiver->setNumSubFrames(mNumSubFrames);

		unsigned int frameDataLengthBytes = this->frameDataLengthBytes();
		mFemDataReceiver->setFrameLength(frameDataLengthBytes);

		bool hasFrameCounter = (reorderMode == reorderedDataMode) ? true : false;
		std::cout << "Setting frame counter mode to " << (hasFrameCounter ? "true" : "false") << std::endl;
		mFemDataReceiver->enableFrameCounter(hasFrameCounter);
		mFemDataReceiver->enableFrameCounterCheck(enableFrameCounterCheck);

		// Start the data receiver thread
		mFemDataReceiver->startAcquisition();

	}
	else
	{
		std::cout << "Data receiver thread is NOT enabled" << std::endl;
	}

	// If the client is in control of this acquisition mode, set up and start the acquisition
	if (clientAcquisitionControl)
	{

		// Setup of shutters and frame counters is dependent on readout mode

		switch(mMpx3OmrParams[0].readWriteMode)
		{
		case sequentialReadWriteMode:
		{
			// Set up the number of frames to be acquired in the ASIC control block
			this->asicControlNumFramesSet(numRxFrames);

			// Set up the acquisition time in the ASIC control block, converting from milliseconds
			// to microseconds. Set both shutters to the same value (why??)
			unsigned int shutterTime = mAcquisitionTimeMs * 1000;
			this->asicControlShutterDurationSet(shutterTime, shutterTime);
		}
			break;

		case continuousReadWriteMode:
		{
			// In continuous mode, force the counter select to start with counter 1
			counterSelect = mpx3Counter1;

			// In this mode, shutter 1 controls the individual frame duration ...
			unsigned int shutter1Time = mAcquisitionTimeMs * 1000;

			// ... and shutter 0 controls the overall acquisition duration, i.e. should be
			// set to the acquisition time multiplied by the number of frames
			unsigned int shutter0Time = (mAcquisitionTimeMs * 1000) * numRxFrames;

			std::cout << "CRW mode, setting shutter 0 duration to " << shutter0Time
					  << "us and shutter 1 duration to " << shutter1Time << "us" << std::endl;
			this->asicControlShutterDurationSet(shutter0Time, shutter1Time);

			// Set frame counter to zero is this mode
			this->asicControlNumFramesSet(0);

		}
			break;

		default:
			{
				std::ostringstream msg;
				msg << "Cannot start acquisition, illegal read write modeh specified: " << mMpx3OmrParams[0].readWriteMode;
				throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalReadWriteMode, msg.str());
			}
			break;

		}

		// Build chip mask from the enable flags and determine which is the first chip active - this is used
		// to select settings for building the OMR for readout transactions, where the OMR is broadcast
		// to all chips
		int firstChipActive = -1;
		unsigned int chipMask = 0;
		for (unsigned int iChip = 0; iChip < kNumAsicsPerFem; iChip++)
		{
			if (mMpx3Enable[iChip])
			{
				chipMask |= ((unsigned int)1 << (7 - iChip));
				if (firstChipActive == -1)
				{
					firstChipActive = iChip;
				}
			}
		}
		std::cout << "Chip mask: 0x" << std::hex << chipMask << std::dec << " First chip active: " << firstChipActive << std::endl;

		// Set up the ASIC mux based on calculated chip mask
		this->asicControlMuxSet(chipMask);

		// Check if test pulses are enabled on any enabled chip, if so set the global test pulse enable flag
		for (unsigned int iChip = 0; iChip < kNumAsicsPerFem; iChip++)
		{
			if (mMpx3Enable[iChip] && mMpx3OmrParams[iChip].testPulseEnable)
			{
				mMpx3GlobalTestPulseEnable = true;
			}
		}

		if (mMpx3GlobalTestPulseEnable)
		{
			std::cout << "Enabling test pulse injection on FEM (count=" << mMpx3TestPulseCount << ")" << std::endl;
			this->asicControlTestPulseCountSet(mMpx3TestPulseCount);
		}

		// Set up OMR mode and execute command based on which counter is selected
		mpx3OMRMode omrMode = (mpx3OMRMode)0;
		switch (counterSelect)
		{
		case mpx3Counter0:
			omrMode    = readPixelMatrixC0;
			break;

		case mpx3Counter1:
			omrMode    = readPixelMatrixC1;
			break;

		default:
			{
				std::ostringstream msg;
				msg << "Cannot start acquisition, illegal counter select specified: " << mMpx3CounterSelect;
				throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalCounterSelect, msg.str());
			}

			break;
		}

		// Set up the OMR for readout using the first active chip to retrieve
		// default values for OMR fields
		mpx3Omr theOmr = this->mpx3OMRBuild(firstChipActive, omrMode);
		//std::cout << "Using MPX OMR: 0x" << std::hex << theOmr.raw << std::dec << std::endl;
		this->asicControlOmrSet(theOmr);

		// Enable test pulses in the execute command if necessary
		if (mMpx3GlobalTestPulseEnable)
		{
			executeCmd |= asicTestPulseEnable;
		}

		// Build the configuration register based on trigger mode and polarity
		unsigned int controlConfigRegister = 0;

		switch (mTriggerMode)
		{
		case excaliburTriggerModeInternal:
			controlConfigRegister |= internalTriggerMode;
			break;

		case excaliburTriggerModeExternal:
			controlConfigRegister |= externalTriggerMode;
			break;

		case excaliburTriggerModeSync:
			controlConfigRegister |= externalSyncMode;
			break;

		default:
			{
				std::ostringstream msg;
				msg << "Cannot start acquisition, illegal trigger mode specified: " << mTriggerMode;
				throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalTriggerMode, msg.str());
			}
			break;
		}

		switch (mTriggerPolarity)
		{
		case excaliburTriggerPolarityActiveHigh:
			controlConfigRegister |= externalTrigActiveHigh;
			break;

		case excaliburTriggerPolarityActiveLow:
			controlConfigRegister |= externalTrigActiveLow;
			break;

		default:
			{
				std::ostringstream msg;
				msg << "Cannot start acquisition, illegal trigger polarity specified: " << mTriggerPolarity;
				throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalTriggerPolarity, msg.str());
			}
			break;
		}

		// Set the control configuration register accordingly
		std::cout << "Setting control configuration register to 0x" << std::hex << controlConfigRegister << std::dec << std::endl;
		this->asicControlConfigRegisterSet(controlConfigRegister);

		// Execute the command
		std::cout << "Sending execute command 0x" << std::hex << executeCmd << std::dec << std::endl;
		this->asicControlCommandExecute((asicControlCommand)executeCmd);

	}
	else
	{
		// Invoke the scan member execute function defined above
		if (theScanFunc != NULL)
		{
			std::cout << "Executing autonomous scan sequence with " << numRxFrames << " steps" << std::endl;
			(this->*(theScanFunc))();
		}
		else
		{
			throw FemClientException((FemClientErrorCode)excaliburFemClientMissingScanFunction, "Missing scan function for this acquisition mode");
		}

	}

	clock_gettime(CLOCK_REALTIME, &endTime);

	double startSecs = startTime.tv_sec  + ((double)startTime.tv_nsec / 1.0E9);
	double endSecs   = endTime.tv_sec  + ((double)endTime.tv_nsec / 1.0E9);
	double elapsedSecs = endSecs - startSecs;

	std::cout << "startAcquisition call took " << elapsedSecs << " secs" << std::endl;
}

void ExcaliburFemClient::stopAcquisition(void)
{

	u32 framesRead = 0;
	bool doFullAcqStop = true;

	// Check if acquisition is active in data receiver. If so, perform the steps necessary to bring the
	// system to a graceful halt. This is dependent on the operation mode currently in force.
	if (mFemDataReceiver != 0)
	{
		if (mFemDataReceiver->acqusitionActive())
		{
			switch (mOperationMode)
			{
			case excaliburOperationModeNormal:
				{
					std::cout << "Normal mode acquisition is still active, sending stop to FEM ASIC control" << std::endl;
					this->asicControlCommandExecute(asicStopAcquisition);

					// Wait at least the acquisition time (shutter time) plus 500us readout time before
					// checking the state to allow last frame to be read out
					usleep((mAcquisitionTimeMs * 1000) + 500);

					// Read control state register for diagnostics
					u32 ctrlState = this->rdmaRead(kExcaliburAsicCtrlState1);
					framesRead = this->rdmaRead(kExcaliburAsicCtrlFrameCount);
					std::cout << "FEM ASIC control has completed " << framesRead
							  << " frames, control state register1: 0x" << std::hex << ctrlState << std::dec << std::endl;

				}
				break;
			case excaliburOperationModeBurst:      // Deliberate fall-thru for these modes where async stop not supported
			case excaliburOperationModeHistogram:
			case excaliburOperationModeMatrixRead:
				std::cout << "Cannot complete asynchronous stop in this operation mode, ignoring stop command while running" << std::endl;
				doFullAcqStop = false;
				break;

			case excaliburOperationModeDacScan:
				{
	#ifdef MPX3_0
					std::cout << "Current FEM firmware does not support asynchronous stop of DAC scan" << std::endl;
					doFullAcqStop = false;
	#else
					std::cout << "Performing asynchronous stop of DAC scan" << std::endl;
					framesRead = this->dacScanAbort();

	#endif
				}
				break;

			default:
			{
				std::ostringstream msg;
				msg << "Cannot stop acquisition, illegal operation mode specified: " << mOperationMode;
				throw FemClientException((FemClientErrorCode)excaliburFemClientIllegalOperationMode, msg.str());
			}
				break;

			} // End of switch

			// Wait until DMA engine has transferred out the number of frames read out by the
			// ASIC control block. This loop will repeat until ten times the frame acquisition
			// length, after which it will assume that the completion has timed out
			bool acqCompletePending = true;
			int numAcqCompleteLoops = 0;
			int maxAcqCompleteLoops = 10;


			while (acqCompletePending && (numAcqCompleteLoops < maxAcqCompleteLoops))
			{
				FemAcquireStatus acqState = this->acquireStatus();
				std::cout << "Asynchronous stop of DMA acquisition loop: " << numAcqCompleteLoops << " attempts, ACQ state: " << acqState.state
						  << " sent BDs: " << acqState.totalSent << std::endl;

				if (acqState.totalSent >= framesRead*2)
				{
					std::cout << "DMA controller has transmitted " << framesRead << " frames OK" << std::endl;
					acqCompletePending = false;
				}
				else
				{
					numAcqCompleteLoops++;
					usleep(mAcquisitionTimeMs * 1000);
				}
			}
			if (acqCompletePending)
			{
				std::cout << "ERROR: DMA transfer of " << framesRead << " failed to complete in expected time during async stop" << std::endl;
			}


		}
	}

	if (doFullAcqStop) {

		// Send ACQUIRE stop command to the FEM
		//std::cout << "Sending acquire STOP to FEM" << std::endl;
		this->acquireStop();

		if (mFemDataReceiver != 0)
		{
			// Ensure that the data receiver has stopped cleanly
			mFemDataReceiver->stopAcquisition(framesRead);

			// Delete the data receiver
			delete(mFemDataReceiver);
			mFemDataReceiver = 0;
		}

		// Reset ASIC control firmware block
		//std::cout << "Sending ASIC control reset to FEM" << std::endl;
		this->asicControlReset();

		//std::cout << "End of doFullAcqStop clause" << std::endl;
	}
}

void ExcaliburFemClient::triggerModeSet(unsigned int aTriggerMode)
{
	// Store trigger mode for use during acquisition start
	mTriggerMode = (excaliburTriggerMode)aTriggerMode;
}

void ExcaliburFemClient::triggerPolaritySet(unsigned int aTriggerPolarity)
{
	// Store trigger polarity for use during acquisition start
	mTriggerPolarity = (excaliburTriggerPolarity)aTriggerPolarity;
}

void ExcaliburFemClient::operationModeSet(unsigned int aOperationMode)
{
	// Store operation mode for use during acquisition start
	mOperationMode = (excaliburOperationMode)aOperationMode;
}

void ExcaliburFemClient::numFramesSet(unsigned int aNumFrames)
{
	// Store number of frames to be received for use during acquisition start
	mNumFrames = aNumFrames;
}

void ExcaliburFemClient::acquisitionPeriodSet(unsigned int aPeriodMs)
{
	// Store acquisition period for use during acquisition start
	mAcquisitionPeriodMs = aPeriodMs;
}

void ExcaliburFemClient::acquisitionTimeSet(unsigned int aTimeMs)
{
	// Store acquisition time for use during acquisition start
	mAcquisitionTimeMs = aTimeMs;
}

void ExcaliburFemClient::burstModeSubmitPeriodSet(double aPeriod)
{

	// Store burst mode submit period for use during acquisition
	mBurstModeSubmitPeriod = aPeriod;
	std::cout << "Set burst mode submit period to " << aPeriod << std::endl;

}

void ExcaliburFemClient::numTestPulsesSet(unsigned int aNumTestPulses)
{
	// Store number of test pulses for use during acquisition start
	mMpx3TestPulseCount = aNumTestPulses;
}

void ExcaliburFemClient::lfsrBypassEnableSet(unsigned int aBypassEnable)
{
	// Store LFSR bypass enable
	mLfsrBypassEnable = (bool)aBypassEnable;
}

unsigned int ExcaliburFemClient::asicReadoutDmaSize(void)
{

	// Get counter bit depth of ASIC
	unsigned int counterBitDepth = this->mpx3CounterBitDepth(mMpx3OmrParams[0].counterDepth);

	// DMA size is (numRows * numCols * (numAsics/2) counterDepth /  8 bits per bytes
	unsigned int theLength = (kNumRowsPerAsic * kNumColsPerAsic * (kNumAsicsPerFem / 2) * counterBitDepth) / 8;

	return theLength;
}

unsigned int ExcaliburFemClient::asicReadoutLengthCycles(void)
{

	unsigned int counterBitDepth = this->mpx3CounterBitDepth(mMpx3OmrParams[0].counterDepth);
	unsigned int readoutBitWidth = this->mpx3ReadoutBitWidth(mMpx3OmrParams[0].readoutWidth);

	unsigned int theLength = (kNumRowsPerAsic * kNumColsPerAsic * counterBitDepth) / readoutBitWidth;

	return theLength;

}

unsigned int ExcaliburFemClient::frameDataLengthBytes(void)
{

	unsigned int frameDataLengthBytes = 0;

	// Get the counter bit depth
	unsigned int counterBitDepth = this->mpx3CounterBitDepth(mMpx3OmrParams[0].counterDepth);

	// Calculate raw length of ASIC data in bits
	unsigned int asicDataLengthBits = kNumRowsPerAsic * kNumColsPerAsic * kNumAsicsPerFem * counterBitDepth;

	// Get the frame length in bytes. In 12/24-bit re-ordered mode, reordering expands each 12 bit ASIC
	// counter up to 16 bits (two bytes). In 6-bit re-ordered mode, reordering expands each 6 bit ASIC up
	// to 8 bits (one byte).
	if (mAsicDataReorderMode == reorderedDataMode)
	{
		switch (mMpx3OmrParams[0].counterDepth)
		{
		case counterDepth1:
			frameDataLengthBytes = asicDataLengthBits / 8; // 1-bit is always forced to raw data mode
			//frameDataLengthBytes = 90112;
			break;
		case counterDepth6:
			frameDataLengthBytes = ((asicDataLengthBits * 8) / 6) / 8;
			break;
		case counterDepth12:
		case counterDepth24:
			frameDataLengthBytes = ((asicDataLengthBits * 16)  / 12) / 8;
			//frameDataLengthBytes = 768432;
			break;

		default:
			break;
		}
	} else {
		frameDataLengthBytes = asicDataLengthBits / 8;
	}

	// Add on size of frame counter(s), which is 8 bytes per subframe
	//frameDataLengthBytes += (mNumSubFrames * 8);

	return frameDataLengthBytes;

}

void ExcaliburFemClient::frontEndInitialise(void)
{

	std::cout << "**** Front-end initialise ****" << std::endl;
	sleep(3);

	// Initialise front-end DACs
	this->frontEndDacInitialise();

	// Reset the ASIC control f/w block and ASICS
	this->asicControlReset();
	this->asicControlAsicReset();

	std::cout << "**** Front-end init done ****" << std::endl;

}

void ExcaliburFemClient::dataReceiverEnable(unsigned int aEnable)
{
	if (aEnable > 0) {
		mDataReceiverEnable = true;
	}
	else {
		mDataReceiverEnable = false;
	}
}

unsigned int ExcaliburFemClient::frameCountGet(void)
{
	return (unsigned int)(this->rdmaRead(kExcaliburAsicCtrlFrameCount - 1));
}

unsigned int ExcaliburFemClient::controlStateGet(void)
{
	return (unsigned int)(this->rdmaRead(kExcaliburAsicCtrlState1));
}
