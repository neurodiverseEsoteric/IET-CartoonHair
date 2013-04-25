#include "stdafx.h"
#include "HairModel.h"
#include "tinyxml2.h"

HairModel::HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world)
{
	tinyxml2::XMLDocument doc;

	//load xml document
	doc.LoadFile(filename);

	tinyxml2::XMLElement *hair = doc.FirstChildElement();

	//get the first strand
	tinyxml2::XMLElement *strand = hair->FirstChildElement();

	//iterate through the strands and save the particles
	for(strand ; strand ; strand = strand->NextSiblingElement())
	{
		//iterate through hair particles - first one is the root and should be fixed in position
		btAlignedObjectArray<btVector3> particles;
		std::vector<float> masses;
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		for(particle ; particle ; particle = particle->NextSiblingElement())
		{
			particles.push_back(btVector3(
				particle->FloatAttribute("x"),
				particle->FloatAttribute("y"),
				particle->FloatAttribute("z")
				));
			masses.push_back(1.0f);
		}

		////now to create the strand softbody
		btSoftBody *hairStrand = createHairStrand(particles,masses,world->getWorldInfo());
		world->addSoftBody(hairStrand);
		m_strandSoftBodies.push_back(hairStrand);

		////clean up
		particles.clear();
		masses.clear();
	}
}

HairModel::~HairModel()
{

}

//based upon lines 508 to 536 of btSoftBodyHelpers.cpp
btSoftBody* HairModel::createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo)
{
	//create softbody
	btSoftBody *strand = new btSoftBody(&worldInfo,particles.size(),&particles[0],&masses[0]);

	//fix root
	strand->setMass(0,0);

	//create links
	for(int node = 1 ; node < particles.size() ; node++)
	{
		strand->appendLink(node-1,node);
	}

	//return strand
	return strand;

}
