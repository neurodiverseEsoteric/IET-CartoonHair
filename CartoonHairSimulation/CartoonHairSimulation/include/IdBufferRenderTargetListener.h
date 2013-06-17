#include "stdafx.h"
#include "HairModel.h"
#include "DebugDrawer.h"

//based on http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
class IdBufferRenderTargetListener : public Ogre::RenderTargetListener
{
public:
	IdBufferRenderTargetListener(Ogre::SceneManager *sceneMgr,HairModel *hairModel,DebugDrawer *debugDrawer, Ogre::Entity *head, Ogre::Entity *character);
	virtual ~IdBufferRenderTargetListener();
private:
	virtual void createScene();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
	virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

	Ogre::SceneNode *m_screenNode;
	Ogre::Rectangle2D *m_screen;
	Ogre::Entity *m_head,*m_character;
	HairModel *m_hairModel;
	DebugDrawer *m_debugDrawer;

	bool m_debugEnabled;
	bool m_normalEnabled;
};