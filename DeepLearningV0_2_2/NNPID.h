

#ifndef NNPID_h
#define NNPID_h

#include "Arduino.h"
#include <math.h>

class NNPID
{
	public:
		NNPID(float LearnRate);
		void calculateOutputOfNeurons();
		void calculateInputOfNeurons();
		void updateWeights();
		void updateSetpoint(float New);
		void updatePosition(float New);
		float Output();
		void Printx();
		void Printu();
		void Printw();
	private:
		const static int numInput = 2;
		const static int numOutput = 1;
		const static int numHidden = 3;
		const static int numNodes = 6;
		const static int memDepth = 2;
		
		float tanhPrime(float x);
		
		float x[numNodes][memDepth];
		float u[numNodes][memDepth];
		float w[9] = {1,1,1,1,1,1,1,1,1};
		float Setpoint;
		float Position;
		float _LearnRate;
};

#endif
