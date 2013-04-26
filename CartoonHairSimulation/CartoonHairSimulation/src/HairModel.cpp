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

	//create manual hair object
	m_hairMesh = sceneMgr->createManualObject("hair");
	m_hairMesh->setDynamic(true);

	//iterate through the strands and save the particles
	for(strand ; strand ; strand = strand->NextSiblingElement())
	{
		//iterate through hair particles - first one is the root and should be fixed in position
		btAlignedObjectArray<btVector3> particles;
		std::vector<float> masses;
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		m_hairMesh->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_STRIP);
		for(particle ; particle ; particle = particle->NextSiblingElement())
		{

			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			particles.push_back(btVector3(x,y,z));
			masses.push_back(1.0f);

			m_hairMesh->colour(1.0,0,0);
			m_hairMesh->position(x,y,z);
		}
		m_hairMesh->end();

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

Ogre::ManualObject* HairModel::getManualObject()
{
	return m_hairMesh;
}

void HairModel::updateManualObject()
{
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		btSoftBody* body = m_strandSoftBodies[section];

		m_hairMesh->beginUpdate(section);

		for(int node = 0 ; node < body->m_nodes.size() ; node++)
		{
			m_hairMesh->position(
				body->m_nodes[node].m_x.x(),
				body->m_nodes[node].m_x.y(),
				body->m_nodes[node].m_x.z()
				);
		}

		m_hairMesh->end();
	}
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
