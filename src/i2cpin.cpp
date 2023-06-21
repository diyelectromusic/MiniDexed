//
// midipin.cpp
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
#include "i2cpin.h"
#include <circle/logger.h>
#include <assert.h>

LOGMODULE ("i2cpin");

CI2CPin::CI2CPin (CI2CMaster *pI2CMaster, unsigned addr, unsigned size)
:	m_nI2CAddr (addr),
	m_nI2CSize (size),
	m_pI2CMaster (pI2CMaster),
	m_nValue (0)
{
}

CI2CPin::~CI2CPin (void)
{
}

boolean CI2CPin::Initialize (void)
{
	assert (m_pI2CMaster);
	if (((m_nI2CAddr >= 0x20) && (m_nI2CAddr <= 0x27)) ||
		((m_nI2CAddr >= 0x38) && (m_nI2CAddr <= 0x3F) && (m_nI2CSize == 8)))
	{
		// PCF8574 (8-bit) / 8575 (16-bit) range is 0x20-0x27.
		// PCF8574A (8-bit) range is 0x38-0x3F.
		// Test access to the module
		u8 Buffer[2];
		if (m_nI2CSize == 8)
		{
			if (m_pI2CMaster->Read (m_nI2CAddr, Buffer, 1) >= 0)
			{
				// Successfully read the value
				m_nValue = (u16)Buffer[0];
				LOGNOTE("I2C IO Expander (8-bit) found at addr 0x%02X", m_nI2CAddr);
				return true;
			}
			else
			{
				// Access failed, so mark as disabled
				LOGNOTE("I2C IO Expander (8-bit) NOT found at addr 0x%02X", m_nI2CAddr);
				m_nI2CAddr = 0;
			}
		}
		else if (m_nI2CSize == 16)
		{
			if (m_pI2CMaster->Read (m_nI2CAddr, Buffer, 2) >= 0)
			{
				// Successfully read the value
				m_nValue = (u16)Buffer[0] | ((u16)Buffer[1]<<8);
				LOGNOTE("I2C IO Expander (16-bit) found at addr 0x%02X", m_nI2CAddr);
				return true;
			}
			else
			{
				// Access failed, so mark as disabled
				LOGNOTE("I2C IO Expander (16-bit) NOT found at addr 0x%02X", m_nI2CAddr);
				m_nI2CAddr = 0;
			}
		}
	}
	else
	{
		// Either not supported address or assume I2C IO disabled
		if (m_nI2CAddr != 0) {
			LOGNOTE("I2C IO Expander config problem: addr=0x%02X, size=%d", m_nI2CAddr, m_nI2CSize);
			m_nI2CAddr = 0;
		}
	}
	return false;
}

bool CI2CPin::IsValidPin (unsigned pin)
{
	if (isI2CPin(pin))
	{
		unsigned i2cpin = PinToI2C(pin);
		if (i2cpin < m_nI2CSize)
		{
			return true;
		}
	}
	return false;
}


unsigned CI2CPin::GetAddress (void)
{
	return m_nI2CAddr;
}


unsigned CI2CPin::Read (unsigned pin)
{
	if (pin == 0)
	{
		// pin is disabled
		return LOW;
	}
	else if (isI2CPin(pin))
	{
		unsigned i2cpin = PinToI2C(pin);
		if (m_nValue & (1<<i2cpin))
		{
			return HIGH;
		}
		else
		{
			return LOW;
		}
	}
	else
	{
		return LOW;
	}
}


void CI2CPin::Update ()
{
	assert (m_pI2CMaster);

	// Read the value from the I2C bus
	if (m_nI2CAddr != 0)
	{
		u8 Buffer[2];
		if (m_nI2CSize == 8)
		{
			if (m_pI2CMaster->Read (m_nI2CAddr, Buffer, 1) >= 0)
			{
				// Successfully read the value
				m_nValue = (u16)Buffer[0];
			}
		}
		else if (m_nI2CSize == 16)
		{
			if (m_pI2CMaster->Read (m_nI2CAddr, Buffer, 2) >= 0)
			{
				// Successfully read the value
				m_nValue = (u16)Buffer[0] | ((u16)Buffer[1]<<8);
				//LOGNOTE("I2C IO Expander value = 0x%04X", m_nValue);
			}
		}
	}
	return;
}
