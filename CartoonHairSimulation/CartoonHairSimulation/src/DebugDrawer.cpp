#include "stdafx.h"
#include "DebugDrawer.h"

DebugDrawer::DebugDrawer(Ogre::SceneManager *sceneMgr)
{
	m_linesObject = sceneMgr->createManualObject("debuglines");
	m_linesObject->setDynamic(true);
	m_initialRun = true;
}

DebugDrawer::~DebugDrawer()
{
	delete m_linesObject;
}

void DebugDrawer::begin()
{
	if(m_initialRun)
	{
		m_initialRun = false;
		m_linesObject->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_LIST);
	}
	else
	{
		m_linesObject->beginUpdate(0);
	}
};

void DebugDrawer::end()
{
	m_linesObject->end();
};

Ogre::ManualObject* DebugDrawer::getLinesManualObject()
{
	return m_linesObject;
}

void DebugDrawer::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & color)
{
	m_linesObject->colour(color.x(),color.y(),color.z());
	m_linesObject->position(from.x(),from.y(),from.z());

	m_linesObject->colour(color.x(),color.y(),color.z());
	m_linesObject->position(to.x(),to.y(),to.z());
}