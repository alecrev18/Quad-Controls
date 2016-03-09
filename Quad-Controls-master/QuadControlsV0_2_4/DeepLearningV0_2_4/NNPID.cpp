

#include "Arduino.h"
#include "NNPID.h"

NNPID::NNPID(float LearnRate, double Control)
{
	_LearnRate = LearnRate;
	_Control = Control;
 
	int i =0;
	int j =0;
	for(i;i<numNodes;i++)
	{
		for(j;j<memDepth;j++)
		{
			x[i][j] = (float) 1+(1/random(-100,100));
			u[i][j] = (float) 1+(1/random(-100,100));       
		}
	}
	
	i=0;
	for(i;i<9;i++)
	{
		w[i] = (float) 1/random(-100,100);
	}
}

float NNPID::tanhPrime(float x)
{
  return (1-tanh(x)*tanh(x));
}

float NNPID::Output()
{
	return x[5][0];
}

void NNPID::calculateOutputOfNeurons()
{
  int i = 0;
  int j = 0;
  for(i=0;i<numNodes;i++)
  {
    for(j;j<memDepth-1;j++)
    {
       x[i][j+1] = x[i][j];
    }
  }

      x[0][0] = tanh(u[0][0]);
      x[1][0] = tanh(u[1][0]);
      x[2][0] = tanh(u[2][0]);
      x[5][0] = tanh(u[5][0]);

      x[3][0] = tanh(u[3][1]+u[3][0]);

      x[4][0] = tanh(u[4][0]-u[4][1]);
     
}

void NNPID::calculateInputOfNeurons()
{
  int i = 0;
  int j = 0;
  for(i;i<numNodes;i++)
  {
    for(j;j<memDepth-1;j++)
    {
      u[i][j+1] = u[i][j];
    }
  }
  
  u[0][0] = Setpoint;
  u[1][0] = Output()*30+1220;
  u[2][0] = x[0][0]*w[0]+x[1][0]*w[3];
  u[3][0] = x[0][0]*w[1]+x[1][0]*w[4];
  u[4][0] = x[0][0]*w[2]+x[1][0]*w[5];
  u[5][0] = x[2][0]*w[6]+x[3][0]*w[7]+x[4][0]*w[8];
  
}

void NNPID::updateWeights()
{
  w[6] = w[6] - _LearnRate*tanhPrime(u[2][0])*(Setpoint-Position)*w[6]*tanhPrime(u[5][0])*x[2][0];
  w[7] = w[7] - _LearnRate*tanhPrime(u[3][0])*(Setpoint-Position)*w[7]*tanhPrime(u[5][0])*x[3][0];
  w[8] = w[8] - _LearnRate*tanhPrime(u[4][0])*(Setpoint-Position)*w[8]*tanhPrime(u[5][0])*x[4][0];

  w[0] = w[0] - _LearnRate*x[0][0]*tanhPrime(u[0][0])*(w[0]*tanhPrime(u[2][0])*w[6]*(Setpoint-_Control));                                                    
  w[1] = w[1] - _LearnRate*x[0][0]*tanhPrime(u[0][0])*(w[1]*tanhPrime(u[3][0])*w[7]*(Setpoint-_Control));
  w[2] = w[2] - _LearnRate*x[0][0]*tanhPrime(u[0][0])*(w[2]*tanhPrime(u[4][0])*w[8]*(Setpoint-_Control));
  
  w[3] = w[3] - _LearnRate*x[1][0]*tanhPrime(u[1][0])*(w[3]*tanhPrime(u[2][0])*w[6]*(Setpoint-_Control));
  w[4] = w[4] - _LearnRate*x[1][0]*tanhPrime(u[1][0])*(w[4]*tanhPrime(u[3][0])*w[7]*(Setpoint-_Control));
  w[5] = w[5] - _LearnRate*x[1][0]*tanhPrime(u[1][0])*(w[5]*tanhPrime(u[4][0])*w[8]*(Setpoint-_Control));
}

void NNPID::updateSetpoint(float New)
{
    Setpoint = New;
}

void NNPID::updatePosition(float New)
{
  Position = New;
}

void NNPID::Printx()
{
  Serial.print("X: ");
  int i=0;
  for(i;i<numNodes;i++)
  {
    Serial.print(x[i][0]);
    Serial.print(" , ");
  }
  Serial.println();
}

void NNPID::Printu()
{
  Serial.print("U: ");
  int i=0;
  for(i;i<numNodes;i++)
  {
    Serial.print(u[i][0]);
    Serial.print(" , ");
  }
  Serial.println();
}

void NNPID::Printw()
{
  Serial.print("W: ");
  int i=0;
  for(i;i<9;i++)
  {
    Serial.print(w[i]);
    Serial.print(" , ");
  }
  Serial.println();
}
