#include "ElytraConfigurator.h"


ElytraConfigurator::ElytraConfigurator()
{
	stabSystem=0;
	pp=pr=py=ip=ir=iy=dp=dr=dy=0;
	ppt=prt=pyt=ipt=irt=iyt=dpt=drt=dyt=0;
}


ElytraConfigurator::~ElytraConfigurator()
{

}

void ElytraConfigurator::dump_to_stdout(TiXmlNode* pParent, unsigned int indent)
{
	
}

void ElytraConfigurator::setStabilizeSystem(String str)
{
	if (str=="PID_SYSTEM")
		stabSystem=PID_SYSTEM;
	if (str=="PID_SYSTEM2")
		stabSystem=PID_SYSTEM2;
	if (str=="MANUAL")
		stabSystem=MANUAL;
	if (str=="FUZZY_PID")
		stabSystem=FUZZY_PID;
	if (str=="FUZZY_PID2")
		stabSystem=FUZZY_PID2;
	if (str=="FUZZY1")
		stabSystem=FUZZY1;
	if (str=="FUZZY2")
		stabSystem=FUZZY2;
	if (str=="FUZZY_NEURAL1")
		stabSystem=FUZZY_NEURAL1;
	if (str=="FUZZY_NEURAL2")
		stabSystem=FUZZY_NEURAL2;
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
				std::string type =  getAttrib(pElem, "type", "PID_SYSTEM");
				//std::cout << type.c_str() << std::endl;
			}
			pElem = pElem->FirstChildElement("pitch");
			if (pElem)
			{
				pp=getAttribReal(pElem, "P", 0.0f);
				ip=getAttribReal(pElem, "I", 0.0f);
				dp=getAttribReal(pElem, "D", 0.0f);
				ppt=getAttribReal(pElem, "P2", 0.0f);
				ipt=getAttribReal(pElem, "I2", 0.0f);
				dpt=getAttribReal(pElem, "D2", 0.0f);
			}
			pElem = pElem->FirstChildElement("roll");
			if (pElem)
			{
				pr=getAttribReal(pElem, "P", 0.0f);
				ir=getAttribReal(pElem, "I", 0.0f);
				dr=getAttribReal(pElem, "D", 0.0f);
				prt=getAttribReal(pElem, "P2", 0.0f);
				irt=getAttribReal(pElem, "I2", 0.0f);
				drt=getAttribReal(pElem, "D2", 0.0f);
			}
			pElem = pElem->FirstChildElement("yaw");
			if (pElem)
			{
				py=getAttribReal(pElem, "P", 0.0f);
				iy=getAttribReal(pElem, "I", 0.0f);
				dy=getAttribReal(pElem, "D", 0.0f);
				pyt=getAttribReal(pElem, "P2", 0.0f);
				iyt=getAttribReal(pElem, "I2", 0.0f);
				dyt=getAttribReal(pElem, "D2", 0.0f);
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
	return StringConverter::parseReal(getAttrib(XMLNode,parameter,"0"));
}

bool ElytraConfigurator::getAttribBool(TiXmlElement *XMLNode, const String &parameter, bool defaultValue)
{
	return getAttrib(XMLNode,parameter,"false")=="true";
}

int ElytraConfigurator::getStabilizeSystem()
{
	return stabSystem;
}

Real ElytraConfigurator::getPPitch(){return pp;}
Real ElytraConfigurator::getPRoll(){return pr;}
Real ElytraConfigurator::getPYaw(){return py;}

Real ElytraConfigurator::getIPitch(){return ip;}
Real ElytraConfigurator::getIRoll(){return ir;}
Real ElytraConfigurator::getIYaw(){return iy;}

Real ElytraConfigurator::getDPitch(){return dp;}
Real ElytraConfigurator::getDRoll(){return dr;}
Real ElytraConfigurator::getDYaw(){return dy;}

Real ElytraConfigurator::getPPitchTilt(){return pp;}
Real ElytraConfigurator::getPRollTilt(){return pr;}
Real ElytraConfigurator::getPYawTilt(){return py;}

Real ElytraConfigurator::getIPitchTilt(){return ipt;}
Real ElytraConfigurator::getIRollTilt(){return irt;}
Real ElytraConfigurator::getIYawTilt(){return iyt;}

Real ElytraConfigurator::getDPitchTilt(){return dpt;}
Real ElytraConfigurator::getDRollTilt(){return drt;}
Real ElytraConfigurator::getDYawTilt(){return dyt;}