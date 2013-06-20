#include "stdafx.h"

class TAMGenerator
{
public:
	TAMGenerator(Ogre::SceneManager* sceneMgr,Ogre::Root *root,int width,int height);
	~TAMGenerator();
private:


	Ogre::SceneManager *m_sceneMgr;
	Ogre::Camera *m_camera;
	Ogre::Rectangle2D *m_screen;
	Ogre::RenderTexture *m_renderTexture;
};