#include "stdafx.h"
#include "IdBufferRenderTargetListener.h"
#include "Constants.h"

IdBufferRenderTargetListener::IdBufferRenderTargetListener(Ogre::SceneManager *sceneMgr)
{
	//setup mini-screen so we can view the id buffer - http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
	m_screen = new Ogre::Rectangle2D(true);
	m_screen->setCorners(0.5f,-0.5f,1.0f,-1.0f);
	m_screen->setBoundingBox(Ogre::AxisAlignedBox(-100000.0f * Ogre::Vector3::UNIT_SCALE, 100000.0f * Ogre::Vector3::UNIT_SCALE));
	m_screen->setMaterial("IETCartoonHair/IdBufferMaterial");

#ifdef DEBUG_VISUALISATION
	m_screenNode = sceneMgr->getRootSceneNode()->createChildSceneNode("screenNode");
	m_screenNode->attachObject(m_screen);
#endif DEBUG_VISUALISATION
}

IdBufferRenderTargetListener::~IdBufferRenderTargetListener()
{
	m_idManualObjects.clear();
	m_idManualObjectsMaterials.clear();
	m_idEntities.clear();
	m_idEntityMaterials.clear();
	m_idManualObjectIdMaterials.clear();
	m_idEntityIdMaterials.clear();
	m_ignoredObjects.clear();
	m_ignoredObjectVisibility.clear();
	m_darkenedManualObjects.clear();
	m_darkenedEntities.clear();
	m_darkenedManualObjectsMaterials.clear();
	m_darkenedEntityMaterials.clear();
}

void IdBufferRenderTargetListener::addObjectToID(Ogre::ManualObject *manualObject, Ogre::String idMaterial)
{
	m_idManualObjects.push_back(manualObject);
	m_idManualObjectsMaterials.push_back(manualObject->getSection(0)->getMaterialName());
	m_idManualObjectIdMaterials.push_back(idMaterial);
}

void IdBufferRenderTargetListener::addObjectToID(Ogre::Entity *entity, Ogre::String idMaterial)
{
	m_idEntities.push_back(entity);
	m_idEntityMaterials.push_back(entity->getSubEntity(0)->getMaterialName());
	m_idEntityIdMaterials.push_back(idMaterial);
}

void IdBufferRenderTargetListener::addObjectToIgnore(Ogre::MovableObject *movableObject)
{
	m_ignoredObjects.push_back(movableObject);
	m_ignoredObjectVisibility.push_back(movableObject->isVisible());
}

void IdBufferRenderTargetListener::addObjectToDarken(Ogre::ManualObject *manualObject)
{
	m_darkenedManualObjects.push_back(manualObject);
	m_darkenedManualObjectsMaterials.push_back(manualObject->getSection(0)->getMaterialName());
}

void IdBufferRenderTargetListener::addObjectToDarken(Ogre::Entity *entity)
{
	m_darkenedEntities.push_back(entity);
	m_darkenedEntityMaterials.push_back(entity->getSubEntity(0)->getMaterialName());
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
	m_screen->setVisible(false);
	//setup id objects
	for(int idIndex = 0 ; idIndex < m_idEntities.size() ; idIndex++)
	{
		m_idEntities[idIndex]->setMaterialName(m_idEntityIdMaterials[idIndex]);
	}

	for(int idIndex = 0 ; idIndex < m_idManualObjects.size() ; idIndex++)
	{
		for(int section = 0 ; section < m_idManualObjects[idIndex]->getNumSections() ; section++)
		{
			m_idManualObjects[idIndex]->setMaterialName(section,m_idManualObjectIdMaterials[idIndex]);
		}
	}

	//darken objects
	for(int darkIndex = 0 ; darkIndex < m_darkenedEntities.size() ; darkIndex++)
	{
		m_darkenedEntities[darkIndex]->setMaterialName("IETCartoonHair/BlackMaterial");
	}

	for(int darkIndex = 0 ; darkIndex < m_darkenedManualObjects.size() ; darkIndex++)
	{
		for(int section = 0 ; section < m_darkenedManualObjects[darkIndex]->getNumSections() ; section++)
		{
			m_darkenedManualObjects[darkIndex]->setMaterialName(section,"IETCartoonHair/BlackMaterial");
		}
	}

	//ignore objects
	for(int ignoreIndex = 0 ; ignoreIndex < m_ignoredObjects.size() ; ignoreIndex++)
	{
		m_ignoredObjectVisibility[ignoreIndex] = m_ignoredObjects[ignoreIndex]->isVisible();
		m_ignoredObjects[ignoreIndex]->setVisible(false);
	}
}

void IdBufferRenderTargetListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
	m_screen->setVisible(true);
	//replace id object materials with originals
	for(int idIndex = 0 ; idIndex < m_idEntities.size() ; idIndex++)
	{
		m_idEntities[idIndex]->setMaterialName(m_idEntityMaterials[idIndex]);
	}

	for(int idIndex = 0 ; idIndex < m_idManualObjects.size() ; idIndex++)
	{
		for(int section = 0 ; section < m_idManualObjects[idIndex]->getNumSections() ; section++)
		{
			m_idManualObjects[idIndex]->setMaterialName(section,m_idManualObjectsMaterials[idIndex]);
		}
	}

	//undarken objects
	for(int darkIndex = 0 ; darkIndex < m_darkenedEntities.size() ; darkIndex++)
	{
		m_darkenedEntities[darkIndex]->setMaterialName(m_darkenedEntityMaterials[darkIndex]);
	}

	for(int darkIndex = 0 ; darkIndex < m_darkenedManualObjects.size() ; darkIndex++)
	{
		for(int section = 0 ; section < m_darkenedManualObjects[darkIndex]->getNumSections() ; section++)
		{
			m_darkenedManualObjects[darkIndex]->setMaterialName(section,m_darkenedManualObjectsMaterials[darkIndex]);
		}
	}

	//stop ignoring objects
	for(int ignoreIndex = 0 ; ignoreIndex < m_ignoredObjects.size() ; ignoreIndex++)
	{
		if(m_ignoredObjectVisibility[ignoreIndex])
		{
			m_ignoredObjects[ignoreIndex]->setVisible(true);
		}
	}

}
