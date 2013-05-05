#include "stdafx.h"

#define SHIFT_X 0.2f

class HairModel
{
public:
	HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world);
	~HairModel();
	Ogre::ManualObject* getManualObject();
	void updateManualObject();
	float getSimulationScale();
private:
	float determineScale(float x);
	void createOrUpdateManualObject(bool update);
	std::vector<btSoftBody*> m_strandSoftBodies;
	std::vector<Ogre::Vector3> m_hairShape;
	float m_simulationScale;
	Ogre::ManualObject *m_hairMesh;
	btSoftBody *createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo);
};