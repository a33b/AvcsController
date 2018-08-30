// Mode.h

#pragma once

///////////////////////////////////////////////////////////////////////////////
// This manages transitions between the states of the app:
// Synchronizing: observing the first N cycles while RPM stabilizes.
// Warming: waiting for oil to warm up before trying to control the cams.
// Running: controlling cams based on crank and cam sensors and feedback loops.
///////////////////////////////////////////////////////////////////////////////
class Mode
{
private:
	int currentMode;
	void BeginSynchronizing();
	void BeginWarming();
	void UpdateWarming();
	void BeginRunning();
	int IsSynchronized();

public:
	// Monitor this many revolutions before we consider ourselves synchronized.
	static const unsigned SyncCountdown = 150; // In theory, 250 would give 10 seconds at 1500 RPM, 6 seconds at 2500. In practice...
	static const int Synchronizing = 1;
	static const int Warming = 2;
	static const int Running = 3;

	void Initialize();
	void Update();
	void Fail(const char *message);
	int GetMode() { return this->currentMode; }
	void ClearScreen();
};

///////////////////////////////////////////////////////////////////////////////
// Shared instance of the Mode class
///////////////////////////////////////////////////////////////////////////////
extern Mode mode;

///////////////////////////////////////////////////////////////////////////////
// Self-tests for the mode code.
///////////////////////////////////////////////////////////////////////////////
void SelfTestMode();