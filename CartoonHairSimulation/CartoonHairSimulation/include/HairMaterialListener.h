//based on http://www.ogre3d.org/tikiwiki/Glow&structure=Cookbook
#include "stdafx.h"

class HairMaterialListener : public Ogre::MaterialManager::Listener
{
public:
	HairMaterialListener()
	{
		m_solidMaterial = Ogre::MaterialManager::getSingleton().create("solidMaterial","Internal");
		m_solidMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
	}
	virtual Ogre::Technique *handleSchemeNotFound(unsigned short schemeIndex, const Ogre::String &schemeName, Ogre::Material *originalMaterial, unsigned short lodIndex, const Ogre::Renderable *rend)
	{
		if(schemeName == "solid")
		{
			return m_solidMaterial->getTechnique(0);
		}
	}
private:
	Ogre::MaterialPtr m_solidMaterial;
};