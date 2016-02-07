#include <Servo.h>

int value = 0;
int motor = 0;

Servo firstESC, secondESC, thirdESC, fourthESC;

void setup()
{
	firstESC.attach(2);        // ESC is connected to port 2
	secondESC.attach(3);       // ESC is connected to port 3
	thirdESC.attach(4);        // ESC is connected to port 4
	fourthESC.attach(5);       // ESC is connected to port 5
	Serial.begin(9600);
}

void loop()
{
	if(Serial.avaialble())
	{
		motor = Serial.parseInt();
		if(motor == 1)
		{
			while(true)
			{
				firstESC.writeMicroseconds(value);
				if(Serial.avaialble())
				{
					value = Serial.parseInt();
					if(value == 0)
					{
						firstESC.writeMicroseconds(value);
						break;
					}
				}
			}
		}
		else if(motor == 2)
		{
			while(true)
			{
				secondESC.writeMicroseconds(value);
				if(Serial.avaialble())
				{
					value = Serial.parseInt();
					if(value == 0)
					{
						secondESC.writeMicroseconds(value);
						break;
					}
				}
			}
		}
		else if(motor == 3)
		{
			while(true)
			{
				thirdESC.writeMicroseconds(value);
				if(Serial.avaialble())
				{
					value = Serial.parseInt();
					if(value == 0)
					{
						thirdESC.writeMicroseconds(value);
						break;
					}
				}
			}
		}
		else if(motor == 4)
		{
			while(true)
			{
				fourthESC.writeMicroseconds(value);
				if(Serial.avaialble())
				{
					value = Serial.parseInt();
					if(value == 0)
					{
						fourthESC.writeMicroseconds(value);
						break;
					}
				}
			}
		}
	}
}