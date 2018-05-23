#include <Thor/include/print.hpp>


/* VERY basic float->string with 4 decimal places of precision */
std::string float2String(float number)
{
	char buff[50];
	std::string out;
	
	const char *tmpSign = (number < 0) ? "-" : "";
	float tmpVal = (number < 0) ? -number : number;

	int tmpInt1 = tmpVal;                  // Get the integer
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer

    // Print as parts, note that you need 0-padding for fractional bit.
	std::sprintf(buff, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
	
	out = buff;
	return out;
}