// Rewritten for PROS CORTEX by jyao6429, original license is below
/* -----------------------------------------------------------------------------
                                  MIT License

                        Copyright (c) 2018 Jason McKinney

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

--------------------------------------------------------------------------------
	NERD_PID.c

	Created:  2016-06-30

	Minor Revisions:
	-	v1.0.0  Initial Release
	- v1.1.0  Changed integral windup algorithm, pidCalculate() outputs in range [-1, 1]

--------------------------------------------------------------------------------
	The author asks that proper attribution be given for this software should the
	source be unavailable (for example, if compiled into a binary/used on a robot).

	The author can be contacted via email at jason_at_jmmckinney_dot_net
	or on the VEX Forum as jmmckinney.
-------------------------------------------------------------------------------- */

#include "main.h"

void pidInit(PID *pid, double Kp, double Ki, double Kd)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Kf = 0.0;
	pid->sigma = 0.0;
	pid->lastValue = 0.0;
	pid->lastTime = millis();
}
void pidInitCopy(PID *pid, PID *toCopy)
{
	pid->Kp = toCopy->Kp;
	pid->Ki = toCopy->Ki;
	pid->Kd = toCopy->Kd;
	pid->Kf = toCopy->Kf;
	pid->sigma = 0.0;
	pid->lastValue = 0.0;
	pid->lastTime = millis();
}
double pidCalculate(PID *pid, double setPoint, double processVariable)
{
	double deltaTime = (millis() - pid->lastTime) * 0.001;
	pid->lastTime = millis();

	double deltaPV = 0.0;

  if (deltaTime > 0)
  {
		deltaPV = (processVariable - pid->lastValue) / deltaTime;
  }

  pid->lastValue = processVariable;

	double error = setPoint - processVariable;

  double output = (error * pid->Kp) + (pid->sigma * pid->Ki) - (deltaPV * pid->Kd) + (setPoint * pid->Kf);

	if (!(fabs(output) >= 1.0 && ((error >= 0 && pid->sigma >= 0) || (error < 0 && pid->sigma < 0))))
  {
    pid->sigma += error * deltaTime;
  }

	output = error * pid->Kp
					+ pid->sigma * pid->Ki
					- deltaPV * pid->Kd
					+ setPoint * pid->Kf;

  if (output > 1.0)
  {
    output = 1.0;
  }

  if (output < -1.0)
  {
    output = -1.0;
  }

	return output;
}
