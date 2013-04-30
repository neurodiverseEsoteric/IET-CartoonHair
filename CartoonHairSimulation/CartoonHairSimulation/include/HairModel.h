#include "stdafx.h"

class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world);
	~HairModel();
	Ogre::ManualObject* getManualObject();
	void updateManualObject();
private:
	void createOrUpdateManualObject(bool update);
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	Ogre::ManualObject *m_hairMesh;
	btSoftBody *createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo);
};