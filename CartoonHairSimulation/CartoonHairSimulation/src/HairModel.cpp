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

HairModel::HairModel(const char* filename, Ogre::SceneManager *sceneMgr, btSoftRigidDynamicsWorld *world,
	btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial)
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
		btSoftBody *hairStrand = createHairStrand(particles,masses,world->getWorldInfo(),edgeMaterial,bendingMaterial,torsionMaterial);
		btSoftBody *ghostStrand = createAndLinkGhostStrand(hairStrand,edgeMaterial,bendingMaterial,torsionMaterial);

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
btSoftBody* HairModel::createHairStrand(btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
	btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial)
{
	//create softbody
	btSoftBody *strand = new btSoftBody(&worldInfo,particles.size(),&particles[0],&masses[0]);

	//create edge links
	for(int node = 1 ; node < particles.size() ; node++)
	{
		strand->appendLink(node-1,node,edgeMaterial);
	}

	//create bending springs
	for(int node = 0 ; node < strand->m_nodes.size()-2 ; node++)
	{
		strand->appendLink(node,node+2,bendingMaterial);
	}

	//http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7465

	//assuming each strand is representing 100 strands or 1 gram
	strand->setTotalMass(0.001);

	//fix root
	strand->setMass(0,0);

	return strand;
}

//http://www.fannieliu.com/hairsim/hairsim.html
btSoftBody* HairModel::createAndLinkGhostStrand(btSoftBody *strand,
	btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial)
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
		ghostStrand->appendLink(node-1,node,bendingMaterial);
	}

	//create edge springs
	for(int node = 0 ; node < ghostStrand->m_nodes.size() ; node++)
	{
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node]),edgeMaterial);
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node+1]),edgeMaterial);
	}

	//create torsion springs
	for(int node = 0 ; node <ghostStrand->m_nodes.size()-1 ; node++)
	{
		ghostStrand->appendLink(&(ghostStrand->m_nodes[node]),&(strand->m_nodes[node+2]),torsionMaterial);
		ghostStrand->appendLink(&(strand->m_nodes[node]),&(ghostStrand->m_nodes[node+1]),torsionMaterial);
	}

	//clean up
	particles.clear();
	masses.clear();

	ghostStrand->setTotalMass(strand->getTotalMass());

	return ghostStrand;
}

void HairModel::createOrUpdateManualObject(bool update)
{

	//iterate through each section (one for each strand) of the hair mesh
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		//variables used to create geometry
		Ogre::Vector3 shapeDir(0,-1,0);
		Ogre::Quaternion rot;
		float scale;

		if(update)
		{
			m_hairMesh->beginUpdate(section);
		}
		else
		{
			m_hairMesh->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_STRIP);
			m_strandVertices.push_back(std::vector<Ogre::Vector3>());
		}

		btSoftBody* body = m_strandSoftBodies[section];

		//generate geometry
		for(int node = 0 ; node < body->m_nodes.size()-1 ; node++)
		{
			float pos = (float)(node)/(body->m_nodes.size());
			scale = determineScale(pos);

			rot = determineRotation(shapeDir,
				Ogre::Vector3(body->m_nodes[node].m_x.x(),body->m_nodes[node].m_x.y(),body->m_nodes[node].m_x.z()),
				Ogre::Vector3(body->m_nodes[node+1].m_x.x(),body->m_nodes[node+1].m_x.y(),body->m_nodes[node+1].m_x.z())
				);

			//generate shape
			//if usual section of hair
			for(int i = 0 ; i < m_hairShape.size() ; i++)
			{
				Ogre::Vector3 vert = rot*m_hairShape[i];
				vert = vert*scale;

				vert = Ogre::Vector3(
					body->m_nodes[node].m_x.x()+vert.x,
					body->m_nodes[node].m_x.y()+vert.y,
					body->m_nodes[node].m_x.z()+vert.z
					);

				//update existing geometry
				if(update)
				{
					int index = (node*m_hairShape.size())+i;
					m_strandVertices[section][index] = vert;
				}
				//creating new geometry
				else
				{
					m_strandVertices[section].push_back(vert);
				}
			}
		}

		//in order to cap the top of the strand and taper the end - the last two vertices will be these points
		Ogre::Vector3 start(body->m_nodes[0].m_x.x(),body->m_nodes[0].m_x.y(),body->m_nodes[0].m_x.z());
		Ogre::Vector3 end(body->m_nodes[body->m_nodes.size()-1].m_x.x(),body->m_nodes[body->m_nodes.size()-1].m_x.y(),body->m_nodes[body->m_nodes.size()-1].m_x.z());

		////update existing geometry
		//if(update)
		//{
		//	int index = (body->m_nodes.size()-1)*m_hairShape.size();
		//	m_strandVertices[section][index] = start;
		//	m_strandVertices[section][index+1] = end;
		//}
		////create new geometry
		//else
		//{
		//	m_strandVertices[section].push_back(start);
		//	m_strandVertices[section].push_back(end);
		//}
		
		//generate manual object

		//vertices + normals
		for(int vert = 0 ; vert < m_strandVertices[section].size() ; vert++)
		{
			m_hairMesh->colour(1.0,0,0);
			m_hairMesh->position(m_strandVertices[section][vert]);
			//m_hairMesh->normal(m_strandNormals[section][vert]);
		}

		//indices
		if(!update)
		{
			//for(int node = 0 ; node < body->m_nodes.size()-1 ; node++)
			//{
			//	int layer1 = node*m_hairShape.size();
			//	int layer2 = (node+1)*m_hairShape.size();

			//	for(int i = 0 ; i < m_hairShape.size() ; i++)
			//	{
			//		//if we are at the edge
			//		if(i == m_hairShape.size()-1)
			//		{
			//			m_hairMesh->triangle(layer2+i,layer1,layer1+i);
			//			m_hairMesh->triangle(layer2+i,layer2,layer1);
			//		}
			//		else
			//		{
			//			m_hairMesh->triangle(layer2+i,layer1+(i+1),layer1+i);
			//			m_hairMesh->triangle(layer2+i,layer2+(i+1),layer1+(i+1));
			//		}
			//	}
			//}

			//cap top

			//taper end
		}

		
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

Ogre::Quaternion HairModel::determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1)
{
	Ogre::Vector3 dir = node1 - node0;
	return up.getRotationTo(dir);
}
