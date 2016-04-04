#pragma once

#include <string>
#include "tinyxml.h"

typedef std::string String;
typedef float Real;

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

private:
	void dump_to_stdout(TiXmlNode* pParent, unsigned int indent = 0);
	TiXmlDocument   *SequenceDoc;
	TiXmlElement   *SequenceRoot;
	int stabSystem;
};

