#include "stdafx.h"
#include "TAMGenerator.h"

TAMGenerator::TAMGenerator(Ogre::SceneManager* sceneMgr,Ogre::Root *root,int width,int height)
{
	m_sceneMgr = root->createSceneManager(Ogre::ST_GENERIC);
	m_camera = m_sceneMgr->createCamera("tamcamera");
	m_sceneMgr->getRootSceneNode()->createChildSceneNode("cameraNode")->attachObject(m_camera);
	
	//setup camera
	m_camera->setProjectionType(Ogre::ProjectionType::PT_ORTHOGRAPHIC);
	m_camera->setOrthoWindow(width,height);
	m_camera->lookAt(Ogre::Vector3(0,0,-1));
	m_camera->setNearClipDistance(0.1);
	m_camera->setFarClipDistance(1000);

	m_screen = new Ogre::Rectangle2D(true);
	m_screen->setCorners(0.5f,1.0f,1.0f,0.5f);
	m_screen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));
	m_screen->setMaterial("BaseWhiteNoLighting");

	sceneMgr->getRootSceneNode()->createChildSceneNode("tamrtt node")->attachObject(m_screen);

	Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual("idBuffer",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,
		width,height,0,Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

	m_renderTexture = texture->getBuffer()->getRenderTarget();
};

TAMGenerator::~TAMGenerator()
{

}