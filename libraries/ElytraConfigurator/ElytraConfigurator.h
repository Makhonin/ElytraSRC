#pragma once

#include <string>
#include "tinyxml.h"
#include "StringConverter.h"



enum { PID_SYSTEM, 
	   PID_SYSTEM2, 
	   MANUAL,
	   FUZZY_PID, 
	   FUZZY_PID2,
	   FUZZY1,
	   FUZZY2,
	   FUZZY_NEURAL1,
	   FUZZY_NEURAL2		
};

class ElytraConfigurator
{
public:
	ElytraConfigurator();
	~ElytraConfigurator();
	void scanSetupFile();
	String getAttrib(TiXmlElement *XMLNode, const String &parameter, const String &defaultValue = "");
	Real getAttribReal(TiXmlElement *XMLNode, const String &parameter, Real defaultValue = 0);
	bool getAttribBool(TiXmlElement *XMLNode, const String &parameter, bool defaultValue = false);
	
	int getStabilizeSystem();

	inline float getPPitch();
	inline float getPRoll();
	inline float getPYaw();

	inline float getIPitch();
	inline float getIRoll();
	inline float getIYaw();

	inline float getDPitch();
	inline float getDRoll();
	inline float getDYaw();

	inline float getPPitchTilt();
	inline float getPRollTilt();
	inline float getPYawTilt();

	inline float getIPitchTilt();
	inline float getIRollTilt();
	inline float getIYawTilt();

	inline float getDPitchTilt();
	inline float getDRollTilt();
	inline float getDYawTilt();

private:
	void setStabilizeSystem(String str);
	float pp,pr,py,ip,ir,iy,dp,dr,dy;
	float ppt,prt,pyt,ipt,irt,iyt,dpt,drt,dyt;
	void dump_to_stdout(TiXmlNode* pParent, unsigned int indent = 0);
	TiXmlDocument   *SequenceDoc;
	TiXmlElement   *SequenceRoot;
	int stabSystem;
};

