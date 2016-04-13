/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.

You may alternatively use this source under the terms of a specific version of
the OGRE Unrestricted License provided you have obtained such a license from
Torus Knot Software Ltd.
-----------------------------------------------------------------------------
*/
#include "StringConverter.h"

typedef std::string String;
typedef float Real;

    //-----------------------------------------------------------------------
    Real StringConverter::parseReal(const String& val)
    {
		// Use istringstream for direct correspondence with toString
		std::istringstream str(val);
		Real ret = 0;
		str >> ret;

        return ret;
    }
    //-----------------------------------------------------------------------
    int StringConverter::parseInt(const String& val)
    {
		// Use istringstream for direct correspondence with toString
		std::istringstream str(val);
		int ret = 0;
		str >> ret;

        return ret;
    }
    //-----------------------------------------------------------------------
    unsigned int StringConverter::parseUnsignedInt(const String& val)
    {
		// Use istringstream for direct correspondence with toString
		std::istringstream str(val);
		unsigned int ret = 0;
		str >> ret;

		return ret;
    }
    //-----------------------------------------------------------------------
    long StringConverter::parseLong(const String& val)
    {
		// Use istringstream for direct correspondence with toString
		std::istringstream str(val);
		long ret = 0;
		str >> ret;

		return ret;
    }
    //-----------------------------------------------------------------------
    unsigned long StringConverter::parseUnsignedLong(const String& val)
    {
		// Use istringstream for direct correspondence with toString
		std::istringstream str(val);
		unsigned long ret = 0;
		str >> ret;

		return ret;
    }
    //-----------------------------------------------------------------------
    bool StringConverter::parseBool(const String& val)
    {
		return (val=="true");
    }


