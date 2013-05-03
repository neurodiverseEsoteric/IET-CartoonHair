#include "stdafx.h"

//http://bulletphysics.org/Bullet/BulletFull/classbtIDebugDraw.html
class DebugDrawer : public btIDebugDraw
{
public:
	DebugDrawer(Ogre::SceneManager *sceneMgr);
	~DebugDrawer();
	void begin();
	void end();
	Ogre::ManualObject *getLinesManualObject();
	virtual void drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & color);
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color){}
	virtual void reportErrorWarning(const char* warningString){}
	virtual void draw3dText(const btVector3& location,const char* textString){}
	virtual void setDebugMode(int debugMode){}
	virtual int	getDebugMode() const {return DBG_DrawWireframe;}
private:
	bool m_initialRun;
	Ogre::ManualObject *m_linesObject;
};