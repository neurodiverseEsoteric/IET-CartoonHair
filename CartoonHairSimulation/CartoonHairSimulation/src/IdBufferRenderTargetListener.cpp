#include "stdafx.h"
#include "IdBufferRenderTargetListener.h"

IdBufferRenderTargetListener::IdBufferRenderTargetListener(Ogre::SceneManager *sceneMgr, HairModel *hairModel, DebugDrawer *debugDrawer)
{
	//setup mini-screen so we can view the id buffer - http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
	m_screen = new Ogre::Rectangle2D(true);
	m_screen->setCorners(0.5f,-0.5f,1.0f,-1.0f);
	m_screen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));
	m_screen->setMaterial("IETCartoonHair/SmallScreenMaterial");

	m_screenNode = sceneMgr->getRootSceneNode()->createChildSceneNode("screenNode");
	m_screenNode->attachObject(m_screen);

	m_hairModel = hairModel;
	m_hairModel->getIdBufferTexture()->addListener(this);

	m_debugDrawer = debugDrawer;
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
	m_hairModel->getEdgeManualObject()->setVisible(false);
}

void IdBufferRenderTargetListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
	if(m_debugEnabled)
	{
		m_debugDrawer->getLinesManualObject()->setVisible(true);
	}
	if(m_normalEnabled)
	{
		m_hairModel->getNormalsManualObject()->setVisible(true);
	}
	m_screen->setVisible(true);
	m_hairModel->getEdgeManualObject()->setVisible(true);
}
