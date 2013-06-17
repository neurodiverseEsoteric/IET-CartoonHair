#include "stdafx.h"
#include "IdBufferRenderTargetListener.h"

IdBufferRenderTargetListener::IdBufferRenderTargetListener(Ogre::SceneManager *sceneMgr, HairModel *hairModel, DebugDrawer *debugDrawer, Ogre::Entity *head, Ogre::Entity *character)
{
	//setup mini-screen so we can view the id buffer - http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
	m_screen = new Ogre::Rectangle2D(true);
	m_screen->setCorners(0.5f,-0.5f,1.0f,-1.0f);
	m_screen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));
	m_screen->setMaterial("IETCartoonHair/IdBufferMaterial");

	m_screenNode = sceneMgr->getRootSceneNode()->createChildSceneNode("screenNode");
	m_screenNode->attachObject(m_screen);

	m_hairModel = hairModel;
	m_hairModel->getIdBufferTexture()->addListener(this);

	m_debugDrawer = debugDrawer;

	m_head = head;
	m_character = character;
}

IdBufferRenderTargetListener::~IdBufferRenderTargetListener()
{
	//delete m_screen;
}

void IdBufferRenderTargetListener::createScene()
{

}

bool IdBufferRenderTargetListener::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	return true;
}

void IdBufferRenderTargetListener::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
	//disable visibility of manual objects
	m_debugEnabled = m_debugDrawer->getLinesManualObject()->isVisible();
	if(m_debugEnabled)
	{
		m_debugDrawer->getLinesManualObject()->setVisible(false);
	}
	m_normalEnabled = m_hairModel->getNormalsManualObject()->isVisible();
	if(m_normalEnabled)
	{
		m_hairModel->getNormalsManualObject()->setVisible(false);
	}
	m_screen->setVisible(false);

	Ogre::ManualObject *hair = m_hairModel->getHairManualObject();
	Ogre::ManualObject *edges = m_hairModel->getEdgeManualObject();

	//change materials
	for(int section = 0; section < hair->getNumSections() ; section++)
	{
		hair->setMaterialName(section,"IETCartoonHair/SolidMaterial");
	}

	for(int section = 0; section < edges->getNumSections() ; section++)
	{
		edges->setMaterialName(section,"IETCartoonHair/SolidSilhouetteMaterial");
	}

	m_head->setMaterialName("IETCartoonHair/BlackMaterial",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	m_character->setMaterialName("IETCartoonHair/BlackMaterial",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}

void IdBufferRenderTargetListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
	//enable visibility
	if(m_debugEnabled)
	{
		m_debugDrawer->getLinesManualObject()->setVisible(true);
	}
	if(m_normalEnabled)
	{
		m_hairModel->getNormalsManualObject()->setVisible(true);
	}
	m_screen->setVisible(true);
	//m_hairModel->getEdgeManualObject()->setVisible(true);

	//change back materials
	Ogre::ManualObject *hair = m_hairModel->getHairManualObject();
	Ogre::ManualObject *edges = m_hairModel->getEdgeManualObject();
	
	for(int section = 0; section < hair->getNumSections() ; section++)
	{
		hair->setMaterialName(section,"IETCartoonHair/HairMaterial");
	}
	for(int section = 0; section < edges->getNumSections() ; section++)
	{
		edges->setMaterialName(section,"IETCartoonHair/EdgeMaterial");
	}
	
	m_character->setMaterialName("jaiqua",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

	m_head->setMaterialName("BaseWhiteNoLighting",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}
