//
// i2cpin.h
//
// MiniDexed - Dexed FM synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _i2cpin_h
#define _i2cpin_h

#include <circle/gpiopin.h>
#include <circle/types.h>
#include <circle/i2cmaster.h>

// Any GPIO pins from the configuration in the following range
// are assumed to be pins on an I2C expander...
// NB: "MIDI pins" take over from 128...
#define I2C_PINS	100
#define I2C_PINS_END	127
#define I2CToPin(c)	(((c)==0)?0:((c)+I2C_PINS))
#define PinToI2C(p)	(((p)>=I2C_PINS)?((p)-I2C_PINS):0)
#define isI2CPin(p)	((((p)>=I2C_PINS)&&((p)<I2C_PINS_END))?1:0)

class CI2CPin
{
public:
	CI2CPin (CI2CMaster *pI2CMaster, unsigned addr, unsigned size);
	~CI2CPin (void);

	boolean Initialize (void);

	// NB: pin=1 to 8 for valid pins; pin=0 means disabled
	unsigned Read (unsigned pin);
	void Update (void);
	
	bool IsValidPin (unsigned pin);
	unsigned GetAddress (void);

private:
	unsigned m_nI2CAddr;
	unsigned m_nI2CSize;
	CI2CMaster *m_pI2CMaster;
	u16 m_nValue;
};

#endif
