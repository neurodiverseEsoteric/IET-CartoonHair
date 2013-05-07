#include "stdafx.h"
#include "HairModel.h"
#include "tinyxml2.h"

//Ogre::Vector3 Hexagon[6] = 
//{
//	Ogre::Vector3(-0.0866,0,-0.05),
//	Ogre::Vector3(0,0,-0.1),
//	Ogre::Vector3(0.0866,0,-0.05),
//	Ogre::Vector3(0.0866,0,0.05),
//	Ogre::Vector3(0,0,0.1),
//	Ogre::Vector3(-0.0866,0,0.05)
//};

HairModel::HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world)
{
	tinyxml2::XMLDocument doc;

	//load xml document
	doc.LoadFile(filename);

	tinyxml2::XMLElement *hair = doc.FirstChildElement();

	//get simulation scale
	m_simulationScale = hair->FloatAttribute("scale");

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

			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			particles.push_back(btVector3(x,y,z));
			masses.push_back(1.0f);
		}

		//now to create the strand softbody and its ghost nodes
		btSoftBody *hairStrand = createHairStrand(particles,masses,world->getWorldInfo());
		btSoftBody *ghostStrand = createAndLinkGhostStrand(hairStrand);

		world->addSoftBody(hairStrand,HAIR_GROUP, BODY_GROUP);
		world->addSoftBody(ghostStrand,GHOST_GROUP,NULL);

		m_strandSoftBodies.push_back(hairStrand);
		m_ghostStrandSoftBodies.push_back(ghostStrand);

		////clean up
		particles.clear();
		masses.clear();
	}

	//create hair shape
	m_hairShape.push_back(Ogre::Vector3(-0.0866,0,-0.05));
	m_hairShape.push_back(Ogre::Vector3(0,0,-0.1));
	m_hairShape.push_back(Ogre::Vector3(0.0866,0,-0.05));
	m_hairShape.push_back(Ogre::Vector3(0.0866,0,0.05));
	m_hairShape.push_back(Ogre::Vector3(0,0,0.1));
	m_hairShape.push_back(Ogre::Vector3(-0.0866,0,0.05));

	//Ogre::Vector3 vert(1,0,0);
	//for(int step = 0 ; step < shapeRes ; step++)
	//{
	//	m_hairShape.push_back(vert);
	//	vert = Ogre::Quaternion(Ogre::Degree(360.0f/shapeRes),Ogre::Vector3::UNIT_Y
	//}

	//create manual hair object
	m_hairMesh = sceneMgr->createManualObject("hair");
	m_hairMesh->setDynamic(true);
	createOrUpdateManualObject(false);
}

HairModel::~HairModel()
{
	//TO DO
}

Ogre::ManualObject* HairModel::getManualObject()
{
	return m_hairMesh;
}

void HairModel::updateManualObject()
{
	createOrUpdateManualObject(true);
}

float HairModel::getSimulationScale()
{
	return m_simulationScale;
}

//based upon lines 508 to 536 of btSoftBodyHelpers.cpp
btSoftBody* HairModel::createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo)
{
	//create softbody
	btSoftBody *strand = new btSoftBody(&worldInfo,particles.size(),&particles[0],&masses[0]);

	//create links
	for(int node = 1 ; node < particles.size() ; node++)
	{
		strand->appendLink(node-1,node);
	}

	//create bending springs
	for(int node = 0 ; node < strand->m_nodes.size()-2 ; node++)
	{
		strand->appendLink(node,node+2);
	}

	//http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7465

	//assuming each strand is representing 100 strands or 1 gram
	strand->setTotalMass(0.001);

	//fix root
	strand->setMass(0,0);

	return strand;
}

//http://www.fannieliu.com/hairsim/hairsim.html
btSoftBody* HairModel::createAndLinkGhostStrand(btSoftBody *strand)
{
	//used in the triangle calculation below - no point doing it more than once
	float tan60 = Ogre::Math::Tan(Ogre::Math::PI/3.0f);
	float halfPI = Ogre::Math::HALF_PI;

	btAlignedObjectArray<btVector3> particles;
	std::vector<float> masses;
	//create ghost nodes - perpendicular to segment
	for(int node = 0 ; node < strand->m_nodes.size()-1 ; node++)
	{
		btVector3 segment = strand->m_nodes[node+1].m_x - strand->m_nodes[node].m_x;

		//find the length of the segment
		float length = segment.length();

		//get the direction vector
		btVector3 direction = segment.normalize();

		//find the mid point
		btVector3 midPoint = strand->m_nodes[node].m_x + direction*(length/2.0f);

		//find the required distance of the ghost particle to the mid-point - it should form the tip of an equilateral triangle
		float dist = tan60*(length/2.0f);

		//find rotation to direction
		Ogre::Vector3 xAxis(1,0,0);
		Ogre::Vector3 normal(0,1,0);
		Ogre::Vector3 dir(direction.x(),direction.y(),direction.z());
		Ogre::Quaternion rot = xAxis.getRotationTo(dir);
		normal = rot*normal;

		btVector3 norm;
		norm.setX(normal.x);
		norm.setY(normal.y);
		norm.setZ(normal.z);

		//create arbitrary normal
		//Find the smallest axis - so that we are not just rotating on the spot
		//btVector3 axis(0,0,0);
		//switch(direction.minAxis())
		//{
		////x
		//case 0:
		//	axis.setX(1);
		//	break;
		////y
		//case 1:
		//	axis.setY(1);
		//	break;
		////z
		//case 3:
		//	axis.setZ(1);
		//	break;
		//}

		//and rotate the direction vector 90 degrees on that axis
		/*direction = direction.rotate(axis,halfPI);*/

		particles.push_back(midPoint+(norm*dist));
		masses.push_back(1);
	}

	btSoftBody *ghostStrand = new btSoftBody(strand->getWorldInfo(),particles.size(),&particles[0],&masses[0]);

	//create bending springs
	for(int node = 1 ; node < ghostStrand->m_nodes.size() ; node++)
	{
		ghostStrand->appendLink(node-1,node);
	}

	//create edge springs
	for(int node = 0 ; node < ghostStrand->m_nodes.size() ; node++)
	{
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node]));
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node+1]));
	}

	//create torsion springs
	for(int node = 0 ; node <ghostStrand->m_nodes.size()-1 ; node++)
	{
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node+2]));
		ghostStrand->appendLink(&(strand->m_nodes[node]),&(ghostStrand->m_nodes[node+1]));
	}

	//clean up
	particles.clear();
	masses.clear();

	ghostStrand->setTotalMass(strand->getTotalMass());

	return ghostStrand;
}

void HairModel::createOrUpdateManualObject(bool update)
{
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		if(update)
		{
			m_hairMesh->beginUpdate(section);
		}
		else
		{
			m_hairMesh->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_TRIANGLE_LIST);
		}

		btSoftBody* body = m_strandSoftBodies[section];

		Ogre::Vector3 *shape1 = new Ogre::Vector3[m_hairShape.size()];
		Ogre::Vector3 *shape2 = new Ogre::Vector3[m_hairShape.size()];

		//determine rotation between direction shape is facing and hair strand
		Ogre::Vector3 shapeDir(0,-1,0);
		Ogre::Vector3 hairDir(
			Ogre::Vector3(body->m_nodes[1].m_x.x(),body->m_nodes[1].m_x.y(),body->m_nodes[1].m_x.z())
			-Ogre::Vector3(body->m_nodes[0].m_x.x(),body->m_nodes[0].m_x.y(),body->m_nodes[0].m_x.z())
			);
		Ogre::Quaternion rot = shapeDir.getRotationTo(hairDir);

		float scale = determineScale(0);

		//create shape 1 out of the loop as we will be normally we will just be swapping shape 1 with shape 2 every loop
		for(int i = 0 ; i < m_hairShape.size() ; i++)
		{
			Ogre::Vector3 vert = rot*m_hairShape[i];
			vert = vert*scale;

			shape1[i] = Ogre::Vector3(
			body->m_nodes[0].m_x.x()+vert.x,
			body->m_nodes[0].m_x.y()+vert.y,
			body->m_nodes[0].m_x.z()+vert.z
				);
		}

		for(int node = 0 ; node < body->m_nodes.size()-1 ; node++)
		{

			//need to calculate direction of node+1 to node+2
			if(node+2<body->m_nodes.size())
			{
				hairDir = Ogre::Vector3(body->m_nodes[node+2].m_x.x(),body->m_nodes[node+2].m_x.y(),body->m_nodes[node+2].m_x.z())
					-Ogre::Vector3(body->m_nodes[node+1].m_x.x(),body->m_nodes[node+1].m_x.y(),body->m_nodes[node+1].m_x.z());
				rot = shapeDir.getRotationTo(hairDir);
			}

			float pos = (float)(node+1)/(body->m_nodes.size());
			scale = determineScale(pos);

			for(int i = 0 ; i < m_hairShape.size() ; i++)
			{
				Ogre::Vector3 vert = rot*m_hairShape[i];
				vert = vert*scale;

				shape2[i] = Ogre::Vector3(
					body->m_nodes[node+1].m_x.x()+vert.x,
					body->m_nodes[node+1].m_x.y()+vert.y,
					body->m_nodes[node+1].m_x.z()+vert.z
					);
			}

			//create triangles
			m_hairMesh->colour(1,0,0);

			int index1;
			int index2;
			//need faces between indices 0-1, 1-2, 2-3, 3-4, 4-5, 5-0 if using a hexagon
			for(int face = 0 ; face < m_hairShape.size() ; face++)
			{

				//ensure the faces wrap around
				if(face!= m_hairShape.size()-1)
				{
					index1 = face;
					index2 = face+1;
				}
				else
				{
					index1 = face;
					index2 = 0;
				}

				//if we are not on the last node
				if(node!=body->m_nodes.size()-2)
				{
					//trangle 1
					m_hairMesh->position(shape1[index1].x,shape1[index1].y,shape1[index1].z);
					m_hairMesh->position(shape1[index2].x,shape1[index2].y,shape1[index2].z);
					m_hairMesh->position(shape2[index1].x,shape2[index1].y,shape2[index1].z);
					//triangle 2
					m_hairMesh->position(shape1[index2].x,shape1[index2].y,shape1[index2].z);
					m_hairMesh->position(shape2[index2].x,shape2[index2].y,shape2[index2].z);
					m_hairMesh->position(shape2[index1].x,shape2[index1].y,shape2[index1].z);
				}
				//if we are on the last node - taper to a point
				else
				{
					//triangle 1
					m_hairMesh->position(shape1[index1].x,shape1[index1].y,shape1[index1].z);
					m_hairMesh->position(shape1[index2].x,shape1[index2].y,shape1[index2].z);
					m_hairMesh->position(body->m_nodes[node+1].m_x.x(),body->m_nodes[node+1].m_x.y(),body->m_nodes[node+1].m_x.z());
				}

				//seal the top - triangle 3
				if(node==0)
				{
					m_hairMesh->position(body->m_nodes[node].m_x.x(),body->m_nodes[node].m_x.y(),body->m_nodes[node].m_x.z());
					m_hairMesh->position(shape1[index2].x,shape1[index2].y,shape1[index2].z);
					m_hairMesh->position(shape1[index1].x,shape1[index1].y,shape1[index1].z);
				}
				/*else if(node==body->m_nodes.size()-2)
				{
					m_hairMesh->position(body->m_nodes[node+1].m_x.x(),body->m_nodes[node+1].m_x.y(),body->m_nodes[node+1].m_x.z());
					m_hairMesh->position(shape2[index1].x,shape2[index1].y,shape2[index1].z);
					m_hairMesh->position(shape2[index2].x,shape2[index2].y,shape2[index2].z);
				}*/
			}

			//swap the shapes to avoid re-calculating shape 2
			delete [] shape1;
			shape1 = shape2;
			shape2 = new Ogre::Vector3[m_hairShape.size()];
		}

		//clean up
		delete[] shape1;
		delete[] shape2;

		m_hairMesh->end();
	}
}

float HairModel::determineScale(float x)
{
	//this uses the technique from A Stylized Cartoon Hair Renderer (a64-shin.pdf)
	//return Ogre::Math::Sin(((x+SHIFT_X)*Ogre::Math::PI)/(1+SHIFT_X));

	//http://www.mathopenref.com/quadraticexplorer.html

	//float func = -1.4*x*x + 0.2*x + 1.1;
	//float func = -4.6*x*x + 2.6*x + 1.1;
	//float func = -5.8*x*x + 3*x + 2.7;
	float func = -13.9*x*x+4.9*x+6.4;

	return Ogre::Math::Abs(func);
}
