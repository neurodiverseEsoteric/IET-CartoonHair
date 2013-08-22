#include "stdafx.h"

/*
This class is used for listening for hair sceme changes in the compositor file. This is necessary for the image space silhouette technique.
It is based upon http://www.ogre3d.org/tikiwiki/Glow&structure=Cookbook
*/
/*
Based upon: http://www.ogre3d.org/tikiwiki/Glow&structure=Cookbook
When using the Ogre compositor, we want to render all objects in a solid colour to prevent lighting from interferring with the
edge detection. The compositor lets us re-render the main scene but it does not give direct access to the object materials. It
does however allow us to change the material schemes for the objects. We can therefore define a solid colour scheme for EVERY
material and ask the compositor to switch it. Unfortunately, if any object does not have that scheme, the whole process doesn't work.
So instead, we use a material listener which is called when a scheme is not found and use it to pass a basic solid colour material.
*/
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
		if(schemeName.compare("solid") == 0)
		{
			return m_solidMaterial->getTechnique(0);
		}
		return originalMaterial->getTechnique(0);
	}
private:
	Ogre::MaterialPtr m_solidMaterial;
};