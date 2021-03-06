#include "stdafx.h"
#include "HairModel.h"
#include "DebugDrawer.h"

/*
	This class is responsible for rendering the hair as a flat shaded mesh using id buffer colours rather than the hairs actual colour.
	This is necessary for determining visibility for silhouettes. This is based on http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
*/
class IdBufferRenderTargetListener : public Ogre::RenderTargetListener
{
public:
	IdBufferRenderTargetListener(Ogre::SceneManager *sceneMgr);
	virtual ~IdBufferRenderTargetListener();

	//determine if the ID buffer preview quad should be visible or not
	void setVisible(bool value);

	//add objects that we want to appear in the ID buffer - e.g. the hair and silhouette edges
	void addObjectToID(Ogre::ManualObject *manualObject, Ogre::String idMaterial);
	void addObjectToID(Ogre::Entity *entity, Ogre::String idMaterial);
	//add objects we want to completely ignore in the ID buffer like the ID buffer preview quad
	void addObjectToIgnore(Ogre::MovableObject *movableObject);
	//add objects we don't want to ignore as the must still be present to obscure something like the head but we don't want to have ID'd
	void addObjectToDarken(Ogre::ManualObject *manualObject);
	void addObjectToDarken(Ogre::Entity *entity);
private:
	virtual void createScene();
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
	virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

	std::vector<Ogre::ManualObject*> m_idManualObjects;
	std::vector<Ogre::String> m_idManualObjectsMaterials;
	std::vector<Ogre::Entity*> m_idEntities;
	std::vector<Ogre::String> m_idEntityMaterials;
	std::vector<Ogre::String> m_idManualObjectIdMaterials;
	std::vector<Ogre::String> m_idEntityIdMaterials;

	std::vector<Ogre::MovableObject*> m_ignoredObjects;
	std::vector<bool> m_ignoredObjectVisibility;

	std::vector<Ogre::ManualObject*> m_darkenedManualObjects;
	std::vector<Ogre::Entity*> m_darkenedEntities;
	std::vector<Ogre::String> m_darkenedManualObjectsMaterials;
	std::vector<Ogre::String> m_darkenedEntityMaterials;

	Ogre::SceneNode *m_screenNode;
	Ogre::Rectangle2D *m_screen;

	bool m_screenVisible;
};