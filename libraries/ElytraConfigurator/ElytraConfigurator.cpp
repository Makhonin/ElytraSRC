#include "ElytraConfigurator.h"


ElytraConfigurator::ElytraConfigurator()
{

}


ElytraConfigurator::~ElytraConfigurator()
{

}

void ElytraConfigurator::dump_to_stdout(TiXmlNode* pParent, unsigned int indent)
{
	
}


void ElytraConfigurator::scanSetupFile()
{
	TiXmlDocument doc("elytra.xml");
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		//printf("Parsing settings from: %s:\n", "elytra.xml");
		TiXmlElement* pElem = doc.FirstChildElement("params");
		if (pElem)
		{
			pElem = pElem->FirstChildElement("stab");
			if (pElem)
			{
				std::string type =  getAttrib(pElem, "type", "normal");
				//std::cout << type.c_str() << std::endl;
			}
		}
	}
	else
	{
		//printf("Failed to load file \"%s\"\n", "elytra.xml");
	}
}

String ElytraConfigurator::getAttrib(TiXmlElement *XMLNode, const String &parameter, const String &defaultValue)
{
	if (XMLNode->Attribute(parameter.c_str()))
		return XMLNode->Attribute(parameter.c_str());
	else
		return defaultValue;
}

Real ElytraConfigurator::getAttribReal(TiXmlElement *XMLNode, const String &parameter, Real defaultValue)
{
	return 0.0f;
}

bool ElytraConfigurator::getAttribBool(TiXmlElement *XMLNode, const String &parameter, bool defaultValue)
{
	return true;
}

int ElytraConfigurator::getStabilizeSystem()
{
	return stabSystem;
}