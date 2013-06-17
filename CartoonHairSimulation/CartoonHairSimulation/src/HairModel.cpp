#include "stdafx.h"
#include "HairModel.h"
#include "tinyxml2.h"

//based on http://content.gpwiki.org/index.php/OpenGL_Selection_Using_Unique_Color_IDs
Ogre::ColourValue HairModel::generateUniqueColour()
{
	Ogre::ColourValue result = m_currentId;
	m_currentId.r += ID_INCREMENT;
	if(m_currentId.r > 1.0f)
	{
		m_currentId.r = 0.0f;
		m_currentId.g += ID_INCREMENT;
		if(m_currentId.g > 1.0f)
		{
			m_currentId.g = 0;
			m_currentId.b += ID_INCREMENT;
		}

	}

	return result;
}

HairModel::HairModel(HairParameters &param)
{
	m_currentId = Ogre::ColourValue(0.01,0.0,0.0,1.0);

	m_camera = param.camera;
	m_a = param.a;
	m_b = param.b;
	m_c = param.c;
	m_world = param.world;

	//m_initialPosition = btVector3(0,0,0);//btVector3(param.initialPosition.x,param.initialPosition.y,param.initialPosition.z);
	//m_initialOrientation = btQuaternion(0,0,0,1);//btQuaternion(param.initialOrientation.x,param.initialOrientation.y,param.initialOrientation.z,param.initialOrientation.w);

	m_maxStictionConnections = param.maxStictionConnections;
	m_stictionThreshold = param.stictionThreshold;
	m_stictionMaterial = param.stictionMaterial;
	m_stictionRestLength = param.stictionRestLength;
	m_stictionK = param.stictionK;

	m_currentFrame = 0;
	m_animationTime = 0;

	m_depthCueCalculated = false;

	//create render texture for storing colour id buffer for silhouette visibility testing
	//http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
	Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual("idBuffer",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,
		param.window->getWidth(),param.window->getHeight(),0,Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);
	m_idBuffer = texture->getBuffer()->getRenderTarget();
	m_idBuffer->addViewport(param.camera);
	m_idBuffer->getViewport(0)->setClearEveryFrame(true);
	m_idBuffer->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
	m_idBuffer->getViewport(0)->setOverlaysEnabled(false);
	m_idBuffer->setAutoUpdated(true);

	std::string hairFrame = loadAnchorPoints(param.directory,param.animation);
	generateAnchorBody(param.world, param.world->getWorldInfo(), m_anchorPoints[0]);
	generateHairStrands(param.directory+hairFrame,param.world,param.edgeMaterial,param.bendingMaterial,param.torsionMaterial);
	generateHairMesh(param.sceneMgr);
}

HairModel::~HairModel()
{
	m_hairShape.clear();
	m_strandVertices.clear();
	m_strandNormals.clear();
	m_strandIndices.clear();
	m_edgeMap.clear();
	m_idColours.clear();

	for(int i = 0 ; i < m_hairSegments.size() ; i++)
	{
		btGhostObject* ghost = m_hairSegments[i]->ghostObject;
		m_world->removeCollisionObject(ghost);
		delete ghost->getCollisionShape();
		delete ghost;
		delete m_hairSegments[i];
	}

	for(int i = 0 ; i < m_ghostStrandSoftBodies.size() ; i++)
	{
		btSoftBody* strand = m_ghostStrandSoftBodies[i];
		m_world->removeSoftBody(strand);
		delete strand;
	}
	m_ghostStrandSoftBodies.clear();

	for(int i = 0 ; i < m_strandSoftBodies.size() ; i++)
	{
		btSoftBody* strand = m_strandSoftBodies[i];
		m_world->removeSoftBody(strand);
		delete strand;
	}
	m_strandSoftBodies.clear();

	for(int i = 0 ; i < m_anchorPoints.size() ; i++)
	{
		m_anchorPoints[i].clear();
	}
	m_anchorPoints.clear();

	m_world->removeSoftBody(m_anchors);
	delete m_anchors;

}

Ogre::ManualObject* HairModel::getHairManualObject()
{
	return m_hairMesh;
}

Ogre::ManualObject* HairModel::getNormalsManualObject()
{
	return m_normalMesh;
}

Ogre::ManualObject* HairModel::getEdgeManualObject()
{
	return m_edgeMesh;
}

Ogre::RenderTexture* HairModel::getIdBufferTexture()
{
	return m_idBuffer;
}

void HairModel::applyHeadTransform(Ogre::Quaternion rotation, Ogre::Vector3 translation)
{
	//convert to bullet physics variables
	//btVector3 translate(translation.x,translation.y,translation.z);
	//btQuaternion orient(rotation.x,rotation.y,rotation.z,rotation.w);

	////m_localTranslation = m_globalTranslation - translate;
	//////m_localOrientation = m_globalOrientation - orient;
	////btTransform transform(btQuaternion(rotation.x,rotation.y,rotation.z,rotation.w),
	////	btVector3(translation.x,translation.y, translation.z));
	for(int strand = 0 ; strand < m_strandSoftBodies.size() ; strand++)
	{
		m_strandSoftBodies[strand]->m_nodes[0].m_x += btVector3(0.01,0,0);
	}

	for(int node = 0 ; node < m_anchors->m_nodes.size() ; node++)
	{
		m_anchors->m_nodes[node].m_x += btVector3(0.01,0,0);
	}
}

void HairModel::updateManualObject()
{
	createOrUpdateManualObject(true);
	generateEdges(true);
}

float HairModel::getSimulationScale()
{
	return m_simulationScale;
}

void HairModel::addStictionSegment(btSoftRigidDynamicsWorld *world, btSoftBody* strand, int nodeIndex0, int nodeIndex1)
{
	//determine the length of the hair
	btVector3 node2Node = strand->m_nodes[nodeIndex1].m_x - strand->m_nodes[nodeIndex0].m_x;
	btVector3 midPoint = (strand->m_nodes[nodeIndex1].m_x + strand->m_nodes[nodeIndex0].m_x)*0.5f;
	float length = node2Node.length();
	float halfExtent = length*0.5f;

	btGhostObject *ghostObject = new btGhostObject();

	//create collision shape - make it a box
	btBoxShape *boxShape = new btBoxShape(btVector3(halfExtent,halfExtent,halfExtent));
	ghostObject->setCollisionShape(boxShape);

	ghostObject->setWorldTransform(btTransform(btQuaternion(0,0,0,1),midPoint));

	world->addCollisionObject(ghostObject,SEGMENT_GROUP,SEGMENT_GROUP);

	HairSegment *segment = new HairSegment();

	segment->ghostObject = ghostObject;
	segment->strand = strand;
	segment->node0Index = nodeIndex0;
	segment->node1Index = nodeIndex1;

	//have the ghost object point to the hair segment so we can later determine what segment is colliding
	ghostObject->setUserPointer(segment);

	m_hairSegments.push_back(segment);
}

void HairModel::updateAnchors(float timestep)
{
	m_animationTime += timestep*m_animationSpeed;

	if(m_animationTime > 1.0f)
	{
		m_animationTime = 0.0f;
		m_currentFrame++;
		if(m_currentFrame>=m_anchorPoints.size())
		{
			m_currentFrame = 0;
		}
	}

	int nextFrame = m_currentFrame+1;
	if(nextFrame>=m_anchorPoints.size())
	{
		nextFrame = 0;
	}

	for(int node = 0 ; node < m_anchorPoints[m_currentFrame].size() ; node++)
	{
		btVector3 p0 = m_anchorPoints[m_currentFrame][node];
		btVector3 p1 = m_anchorPoints[nextFrame][node];

		btVector3 currentPosition = p0 + (p1-p0)*m_animationTime;
		m_anchors->m_nodes[node].m_x = currentPosition;
	}
}

void HairModel::updateStictionSegments()
{
	for(int segment = 0 ; segment < m_hairSegments.size() ; segment++)
	{
		HairSegment *currentSegment = m_hairSegments[segment];
		btSoftBody *strand = currentSegment->strand;

		btVector3 node2Node = strand->m_nodes[currentSegment->node1Index].m_x - strand->m_nodes[currentSegment->node0Index].m_x;
		btVector3 midPoint = (strand->m_nodes[currentSegment->node1Index].m_x + strand->m_nodes[currentSegment->node0Index].m_x)*0.5f;

		btGhostObject *ghost = currentSegment->ghostObject;
		ghost->setWorldTransform(btTransform(btQuaternion(0,0,0,1),midPoint));

		//disable stiction springs for now
		continue;

		//create stiction springs
		for(int obj = 0 ; obj < ghost->getNumOverlappingObjects() ; obj++)
		{
			HairSegment *neighbourSegment = (HairSegment*)ghost->getOverlappingObject(obj)->getUserPointer();
			//make sure it is not the same strand
			if(neighbourSegment->strand != currentSegment->strand)
			{
				btVector3 point0, point1;
				getClosestPoints(
					currentSegment->strand->m_nodes[currentSegment->node0Index].m_x,
					currentSegment->strand->m_nodes[currentSegment->node1Index].m_x,
					neighbourSegment->strand->m_nodes[neighbourSegment->node0Index].m_x,
					neighbourSegment->strand->m_nodes[neighbourSegment->node1Index].m_x,
					point0,
					point1
					);

				float length = (point1-point0).length();
				if(length < m_stictionThreshold)
				{
					btVector3 force = (length - m_stictionRestLength)*m_stictionK*(point1-point0).normalize();
					neighbourSegment->strand->addForce(force,neighbourSegment->node0Index);
					neighbourSegment->strand->addForce(force,neighbourSegment->node1Index);
				}
			}
		}
	}
}

//based on calculations from http://www.fannieliu.com/hairsim/hairsim.html and http://physbam.stanford.edu/~fedkiw/papers/stanford2002-01.pdf
void HairModel::getClosestPoints(const btVector3 &strand0p0,const btVector3 &strand0p1, const btVector3 &strand1p0, const btVector3 &strand1p1, btVector3 &point0, btVector3 &point1)
{
	glm::vec3 s0p0(strand0p0.x(),strand0p0.y(),strand0p0.z());
	glm::vec3 s0p1(strand0p1.x(),strand0p1.y(),strand0p1.z());
	glm::vec3 s1p0(strand1p0.x(),strand1p0.y(),strand1p0.z());
	glm::vec3 s1p1(strand1p1.x(),strand1p1.y(),strand1p1.z());

	glm::vec3 p0,p1;

	glm::vec3 x21 = s0p1 - s0p0;
	glm::vec3 x31 = s1p0 - s0p0;
	glm::vec3 x43 = s1p1 - s1p0;

	float a = 0.0f, b = 0.0f;
	if(glm::length(glm::cross(x21,x43))>TOLERANCE)
	{
		float dx21x21 = glm::dot(x21,x21);
		float dmx21x43 = glm::dot(-x21,x43);
		float dx43x43 = glm::dot(x43,x43);
		float dx21x31 = glm::dot(x21,x31);
		float dmx43x31 = glm::dot(-x43,x31);

		//http://en.wikipedia.org/wiki/Linear_least_squares_(mathematics)
		//we are trying to solve the linear least squares problem X*beta = y
		glm::mat2x2 X( 
			dx21x21,dmx21x43,
			dmx21x43,dx43x43
			);

		glm::vec2 y(
			dx21x31,
			dmx43x31
			);

		glm::mat2x2 Xt = glm::transpose(X);

		glm::vec2 beta = glm::inverse(Xt*X)*Xt*y;
		
		a = beta.x;
		b = beta.y;

		//clamp
		if(a>1)
		{
			a = 1;
		}
		else if(a<0)
		{
			a = 0;
		}
		if(b>1)
		{
			b = 1;
		}
		else if(b<0)
		{
			b = 0;
		}
	}

	p0 = s0p0 + a*x21;
	p1 = s1p0 + b*x43;

	point0.setValue(p0.x,p0.y,p0.z);
	point1.setValue(p1.x,p1.y,p1.z);
}

//loads all of the specified hair frames into a vector of vectors for anchor animation and returns the first frame's location
std::string HairModel::loadAnchorPoints(std::string directory, std::string filename)
{
	tinyxml2::XMLDocument doc;

	doc.LoadFile((directory+filename).c_str());

	m_animationSpeed = doc.FirstChildElement()->FloatAttribute("speed");

	//get names of the hair frames
	tinyxml2::XMLElement *hair = doc.FirstChildElement()->FirstChildElement();

	std::string firstFrame(hair->Attribute("name"));

	for(hair; hair ; hair = hair->NextSiblingElement())
	{
		//load vertices
		std::string name(hair->Attribute("name"));
		m_anchorPoints.push_back(loadAnchorPositions(directory+name));
	}

	return firstFrame;
}

btAlignedObjectArray<btVector3> HairModel::loadAnchorPositions(std::string filename)
{
	btAlignedObjectArray<btVector3> vertices;

	tinyxml2::XMLDocument doc;

	//load xml document
	doc.LoadFile(filename.c_str());

	tinyxml2::XMLElement *hair = doc.FirstChildElement();

	//get the first strand
	tinyxml2::XMLElement *strand = hair->FirstChildElement();

	//iterate through the strands and save the particles
	for(strand ; strand ; strand = strand->NextSiblingElement())
	{
		//iterate through hair particles - first one is the root and should be fixed in position
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		for(particle ; particle ; particle = particle->NextSiblingElement())
		{
			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			btVector3 point(x,y,z);
			//point = point + m_initialPosition;

			vertices.push_back(point);
		}
	}

	return vertices;
}

void HairModel::generateAnchorBody(btSoftRigidDynamicsWorld *world, btSoftBodyWorldInfo &worldInfo, btAlignedObjectArray<btVector3> &points)
{
	//need to set all masses to 0 so that the nodes are static
	std::vector<float> masses;
	for(int i = 0 ; i < points.size() ; i++)
	{
		masses.push_back(0.0f);
	}

	m_anchors = new btSoftBody(&worldInfo,points.size(),&points[0],&masses[0]);
	world->addSoftBody(m_anchors,NULL,NULL);
}

void HairModel::generateHairStrands(std::string filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial)
{
	tinyxml2::XMLDocument doc;

	//load xml document
	doc.LoadFile(filename.c_str());

	tinyxml2::XMLElement *hair = doc.FirstChildElement();

	//get simulation scale
	m_simulationScale = hair->FloatAttribute("scale");

	//get the first strand
	tinyxml2::XMLElement *strand = hair->FirstChildElement();

	//need to keep track of the strand count so we can index into the anchor mesh node correctly
	int strandCount = 0;
	//iterate through the strands and save the particles
	for(strand ; strand ; strand = strand->NextSiblingElement())
	{
		//iterate through hair particles - first one is the root and should be fixed in position
		btAlignedObjectArray<btVector3> particles;
		//std::vector<btRigidBody*> anchors;

		std::vector<float> masses;
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		for(particle ; particle ; particle = particle->NextSiblingElement())
		{

			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			btVector3 point(x,y,z);
			//point = point + m_initialPosition;

			particles.push_back(point);
			masses.push_back(1.0f);

			//create anchor at this position - http://www.oogtech.org/content/2011/09/07/bullet-survival-kit-4-the-motion-state/
			//we don't need a collision shape
			//btEmptyShape *emptyShape = new btEmptyShape();
			//btDefaultMotionState *anchorMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
			//btRigidBody::btRigidBodyConstructionInfo anchorConstructionInfo(0,anchorMotionState,emptyShape,btVector3(0,0,0));
			//btRigidBody *anchor = new btRigidBody(anchorConstructionInfo);
			//anchor->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(x,y,z)));
			//anchor->setCollisionFlags(anchor->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			//anchor->setActivationState(DISABLE_DEACTIVATION);

			//anchors.push_back(anchor);
			//m_anchors.push_back(anchor);

			//m_world->addCollisionObject(anchor,NULL,NULL);
		}

		//now to create the strand softbody and its ghost nodes
		btSoftBody *hairStrand = createHairStrand(strandCount,world,particles,masses,world->getWorldInfo(),edgeMaterial,bendingMaterial,torsionMaterial);
		btSoftBody *ghostStrand = createAndLinkGhostStrand(hairStrand,edgeMaterial,bendingMaterial,torsionMaterial);

		world->addSoftBody(hairStrand,HAIR_GROUP, BODY_GROUP);
		world->addSoftBody(ghostStrand,GHOST_GROUP,NULL);

		m_strandSoftBodies.push_back(hairStrand);
		m_ghostStrandSoftBodies.push_back(ghostStrand);

		////clean up
		particles.clear();
		masses.clear();

		strandCount++;
	}
}

void HairModel::generateHairMesh(Ogre::SceneManager *sceneMgr)
{
	//create hair shape
	Ogre::Vector3 vertex(0,0,-0.1);
	Ogre::Radian rad(Ogre::Degree(-360.0f/NUM_HAIR_SHAPE_SAMPLES));
	m_hairShape.push_back(vertex);
	for(int sample = 1 ; sample < NUM_HAIR_SHAPE_SAMPLES ; sample++)
	{
		vertex = Ogre::Quaternion(rad,Ogre::Vector3::UNIT_Y)*vertex;
		m_hairShape.push_back(vertex);
	}

	//create manual hair object
	m_hairMesh = sceneMgr->createManualObject("hair");
	m_hairMesh->setDynamic(true);

	m_normalMesh = sceneMgr->createManualObject("normals");
	m_normalMesh->setDynamic(true);

	m_edgeMesh = sceneMgr->createManualObject("edges");
	m_edgeMesh->setDynamic(true);

	//m_edgeSet = sceneMgr->createBillboardSet("edges");

	createOrUpdateManualObject(false);
	generateEdges(false);
}

//based upon lines 508 to 536 of btSoftBodyHelpers.cpp
btSoftBody* HairModel::createHairStrand(int strandIndex, btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
	btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial)
{
	//create softbody
	btSoftBody *strand = new btSoftBody(&worldInfo,particles.size(),&particles[0],&masses[0]);

	btSoftBody::Material *testMaterial = new btSoftBody::Material();
	testMaterial->m_kLST = 0.001f;

	//attach anchor points
	for(int node = 0 ; node < particles.size() ; node++)
	{
		strand->appendLink(&(strand->m_nodes[node]),&(m_anchors->m_nodes[strandIndex*particles.size()+node]),testMaterial);
	}

	//create edge links
	for(int node = 1 ; node < particles.size() ; node++)
	{
		strand->appendLink(node-1,node,edgeMaterial);

		//generate stiction segment
		//addStictionSegment(world,strand,node-1,node);
	}

	//create bending springs
	for(int node = 0 ; node < strand->m_nodes.size()-2 ; node++)
	{
		strand->appendLink(node,node+2,bendingMaterial);
	}

	//attach to anchors - 01309193.pdf
	/*for(int node = 0 ; node < strand->m_nodes.size() ; node++)
	{
		strand->appendAnchor(node,anchors[node],true,0.1f);
	}*/

	//static node test
	//strand->appendNode(btVector3(0,0,0),0.0f);

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
		btVector3 midPoint = (strand->m_nodes[node+1].m_x + strand->m_nodes[node].m_x)*0.5f;

		//find the required distance of the ghost particle to the mid-point - it should form the tip of an equilateral triangle
		float dist = tan60*(length*0.5f);

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

void HairModel::generateIndices()
{
	int numNodes = NUM_HAIR_SAMPLES;//m_strandSoftBodies[0]->m_nodes.size();
	int numVerts = m_hairShape.size();

	//for the normal sides of the strand
	for(int nodes = 0 ; nodes < numNodes-2 ; nodes++)
	{
		for(int vert = 0 ; vert < numVerts ; vert++)
		{
			/*
			Here is a rough idea of how the indices are laid out
			1-2
			|/|
			3-4
			*/
			int index1 = (nodes*numVerts)+vert;
			int index2 = index1 + 1;
			////stop the the index from wrapping around
			if(index2 == (nodes*numVerts)+numVerts)
			{
				index2 = (nodes*numVerts);
			}
			int index3 = index1+numVerts;
			int index4 = index2+numVerts;
		
			//trangle 1
			m_strandIndices.push_back(index1);
			m_strandIndices.push_back(index2);
			m_strandIndices.push_back(index3);
			//triangle 2
			m_strandIndices.push_back(index4);
			m_strandIndices.push_back(index3);
			m_strandIndices.push_back(index2);
		}
	}

	//for the top
	int topIndex = (numNodes-1)*m_hairShape.size();
	for(int vert = 0 ; vert < m_hairShape.size() ; vert++)
	{
		int index1 = vert;
		int index2 = (index1==m_hairShape.size()-1)?0:vert+1;
		m_strandIndices.push_back(topIndex);
		m_strandIndices.push_back(index2);
		m_strandIndices.push_back(index1);
	}
	//for the tip
	int tipIndex = topIndex+1;
	for(int vert = 0 ; vert < m_hairShape.size() ; vert++)
	{
		int offset = (numNodes-2)*m_hairShape.size();
		int index1 = vert;
		int index2 = (index1==m_hairShape.size()-1)?0:vert+1;
		m_strandIndices.push_back(tipIndex);
		m_strandIndices.push_back(index1+offset);
		m_strandIndices.push_back(index2+offset);
	}
}

void HairModel::generateVertices(bool update, int section)
{
	btSoftBody* body = m_strandSoftBodies[section];

	//update spline
	for(int node = 0 ; node < body->m_nodes.size() ; node++)
	{
		btVector3 pos = body->m_nodes[node].m_x;
		Ogre::Vector3 position(pos.x(),pos.y(),pos.z());
		if(update)
		{
			m_hairSplines[section].updatePoint(node,position);
		}
		else
		{
			m_hairSplines[section].addPoint(position);
		}
	}

	m_hairSplines[section].recalcTangents();

	//variables used to create geometry
	Ogre::Vector3 shapeDir(0,-1,0);
	Ogre::Quaternion rot;
	float scale;

	float increment = 1.0f/NUM_HAIR_SAMPLES;

	for(int node = 0 ; node < NUM_HAIR_SAMPLES-1 ; node++)
	{
		float t = node*increment;
		scale = determineScale(t);

		Ogre::Vector3 nodei = m_hairSplines[section].interpolate(t);
		Ogre::Vector3 nodei_1 = m_hairSplines[section].interpolate(t+increment);

		rot = determineRotation(shapeDir,nodei,nodei_1);

		//generate shape
		//if usual section of hair
		for(int i = 0 ; i < m_hairShape.size() ; i++)
		{
			Ogre::Vector3 vert = rot*m_hairShape[i];
			vert = vert*scale;

			vert = Ogre::Vector3(
				nodei.x+vert.x,
				nodei.y+vert.y,
				nodei.z+vert.z
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
	Ogre::Vector3 start(m_hairSplines[section].interpolate(0));
	Ogre::Vector3 end(m_hairSplines[section].interpolate(1));

	if(update)
	{
		int index = (NUM_HAIR_SAMPLES-1)*m_hairShape.size();
		m_strandVertices[section][index] = start;
		m_strandVertices[section][index+1] = end;
	}
	else
	{
		m_strandVertices[section].push_back(start);
		m_strandVertices[section].push_back(end);
	}
}

void HairModel::generateNormals(bool update, int section)
{
	//reset normals
	for(int vert = 0 ; vert < m_strandVertices[section].size() ; vert++)
	{
		if(update)
		{
			m_strandNormals[section][vert] = Ogre::Vector3(0,0,0);
		}
		else
		{
			m_strandNormals[section].push_back(Ogre::Vector3(0,0,0));
		}
	}

	//generate normals - http://assimp.svn.sourceforge.net/viewvc/assimp/trunk/code/GenVertexNormalsProcess.cpp?revision=661&view=markup
	for(int face = 0 ; face < m_strandIndices.size() ; face+=3)
	{
		int v1Index = m_strandIndices[face];
		int v2Index = m_strandIndices[face+1];
		int v3Index = m_strandIndices[face+2];

		Ogre::Vector3 v1 = m_strandVertices[section][v1Index];
		Ogre::Vector3 v2 = m_strandVertices[section][v2Index];
		Ogre::Vector3 v3 = m_strandVertices[section][v3Index];

		Ogre::Vector3 normal = calculateNormal(v1,v2,v3);

		m_strandNormals[section][v1Index] += normal;
		m_strandNormals[section][v2Index] += normal;
		m_strandNormals[section][v3Index] += normal;
	}

	//normalise
	for(int normals = 0 ; normals < m_strandNormals[section].size() ; normals++)
	{
		m_strandNormals[section][normals].normalise();
	}
}

void HairModel::addToTempEdgeMap(std::pair<int,int> key, int index1, int index2, int index3)
{
	Edge edge = m_tempEdgeMap[key];
	if(edge.edgeCount == 0)
	{
		edge.triangle1Indices[0] = index1;
		edge.triangle1Indices[1] = index2;
		edge.triangle1Indices[2] = index3;
	}
	else if (edge.edgeCount == 1)
	{
		edge.triangle2Indices[0] = index1;
		edge.triangle2Indices[1] = index2;
		edge.triangle2Indices[2] = index3;
	}
	else
	{
		assert(edge.edgeCount < 2);
	}
	edge.edgeCount++;
	m_tempEdgeMap[key] = edge;
}

void HairModel::generateIdColours()
{
	for(int strand = 0 ; strand < m_strandSoftBodies.size() ; strand++)
	{
		m_idColours.push_back(generateUniqueColour());
	}
}

void HairModel::generateEdgeMap()
{
	//go through every triangle
	for(int index = 0 ; index < m_strandIndices.size() ; index+=3)
	{
		int i1 = m_strandIndices[index];
		int i2 = m_strandIndices[index+1];
		int i3 = m_strandIndices[index+2];

		std::pair<int,int> key;
			
		//edge 1
		key.first = i1;
		key.second = i2;

		addToTempEdgeMap(key,i1,i2,i3);

		//edge 2
		key.first = i1;
		key.second = i3;

		addToTempEdgeMap(key,i1,i2,i3);

		//edge 3
		key.first = i2;
		key.second = i3;

		addToTempEdgeMap(key,i1,i2,i3);
	}

	//organised edge map
	std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction>::iterator it;
	for(it = m_tempEdgeMap.begin() ; it != m_tempEdgeMap.end() ; it++)
	{
		std::pair<int,int> pair = it->first;
		//make sure lhs<rhs
		if(pair.first > pair.second)
		{
			int temp = pair.first;
			pair.first = pair.second;
			pair.second = temp;
		}

		m_edgeMap[pair] = it->second;
	}
	//std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction>::iterator current,link;
	////link edges together
	//for(current = m_edgeMap.begin() ; current != m_edgeMap.end() ; current++)
	//{
	//	std::pair<int,int> currentEdge = current->first;
	//	for(link = m_edgeMap.begin() ; link != m_edgeMap.end() ; link++)
	//	{
	//		//make sure it is not the same edge
	//		if(current!=link)
	//		{
	//			std::pair<int,int> possibleEdge = link->first;
	//			if(currentEdge.first == possibleEdge.first ||
	//				currentEdge.first == possibleEdge.second ||
	//				currentEdge.second == possibleEdge.first ||
	//				currentEdge.second == possibleEdge.second)
	//			{
	//				current->second.linkedEdges.push_back(possibleEdge);
	//			}
	//		}
	//	}
	//}
}

bool HairModel::isSilhouette(Edge *edge,int section, Ogre::Vector3 eyeVector)
{
	//calculate normals
	//triangle 1
	Ogre::Vector3 v1 = m_strandVertices[section][edge->triangle1Indices[0]];
	Ogre::Vector3 v2 = m_strandVertices[section][edge->triangle1Indices[1]];
	Ogre::Vector3 v3 = m_strandVertices[section][edge->triangle1Indices[2]];
	Ogre::Vector3 normal1 = calculateNormal(v1,v2,v3);

	//triangle 2
	v1 = m_strandVertices[section][edge->triangle2Indices[0]];
	v2 = m_strandVertices[section][edge->triangle2Indices[1]];
	v3 = m_strandVertices[section][edge->triangle2Indices[2]];
	Ogre::Vector3 normal2 = calculateNormal(v1,v2,v3);

	//npar2000_lake_et_al.pdf
	if(normal1.dotProduct(eyeVector)*normal2.dotProduct(eyeVector)<=0)
	{
		return true;
	}
	return false;
}

void HairModel::generateEdges(bool update)
{
	Ogre::Vector3 eyeVector = m_camera->getDerivedDirection();

	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		if(update)
		{
			m_edgeMesh->beginUpdate(section);
		}
		else
		{
			m_edgeMesh->begin("IETCartoonHair/EdgeMaterial",Ogre::RenderOperation::OT_TRIANGLE_STRIP);
		}

		m_edgeMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("width",(float)m_camera->getViewport()->getActualWidth());
		m_edgeMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("height",(float)m_camera->getViewport()->getActualHeight());

		std::deque<std::pair<int,int>> silhouette;
		std::vector<std::pair<int,int>> temp;

		std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction>::iterator it;
		for(it = m_edgeMap.begin() ; it != m_edgeMap.end() ; it++)
		{
			std::pair<int,int> currentPair = it->first;
			Edge *edge = &(it->second);

			//classify
			if(edge->edgeCount == 1)
			{
				edge->flag = EdgeType::BORDER;
			}
			else if (edge->edgeCount == 2)
			{
				//check if silhouette
				if(isSilhouette(edge,section,eyeVector))
				{
					edge->flag = EdgeType::SILHOUETTE;
				}
				else
				{
					edge->flag = EdgeType::NOTHING;
				}
			}
			else
			{
				edge->flag = EdgeType::NOTHING;
			}

			//add to silhouette vector (should use some sort of insertion sort)
			if(edge->flag == EdgeType::SILHOUETTE)
			{
				//elements.push_back(currentPair);
				insertSilhouette(currentPair,temp,silhouette);
			}

		}

		//calculate depth cue scaling values - only need first section
		//artistic-sils-300dpi.pdf
		//this should be measured from the origin of the mesh but as it is a softbody it doesn't really have an origin
		//so I will just use the first segment for measurements
		if(section == 0)
		{
			btSoftBody *strand = m_strandSoftBodies[0];
			//calculate intial depth ratio
			btVector3 edge1 = strand->m_nodes[1].m_x-strand->m_nodes[0].m_x;

			Ogre::Vector3 tp0,tp1,p0,p1;
			p0 = Ogre::Vector3(strand->m_nodes[0].m_x.x(),strand->m_nodes[0].m_x.y(),strand->m_nodes[0].m_x.z());
			p1 = Ogre::Vector3(strand->m_nodes[1].m_x.x(),strand->m_nodes[1].m_x.y(),strand->m_nodes[1].m_x.z());

			toDeviceCoordinates(tp0,p0,m_camera);
			toDeviceCoordinates(tp1,p1,m_camera);

			Ogre::Vector3 edge2 = tp1-tp0;
			if(m_depthCueCalculated == false)
			{
				m_depthCueCalculated = true;
				m_di = edge1.length()/edge2.length();
				m_dc = m_di;
			}
			else
			{
				m_dc = edge1.length()/edge2.length();
			}
			m_fd = sqrt(m_dc/m_di);
		}

		//we now have the silhouettes - now to convert the points to device coordinates
		//based off of artistic-sils-300dpi.pdf
		//and
		//http://www.ogre3d.org/tikiwiki/tiki-index.php?page=GetScreenspaceCoords
		float zMin = std::numeric_limits<float>::max();
		float zMax = std::numeric_limits<float>::min();
		std::vector<Ogre::Vector3> screenSpacePoints;
		for(int i = 0 ; i < silhouette.size() ; i++)
		{
			Ogre::Vector3 result;
			if(toDeviceCoordinates(result,m_strandVertices[section][silhouette[i].first],m_camera))
			{
				if(result.z>zMax)
				{
					zMax = result.z;
				}
				if(result.z<zMin)
				{
					zMin = result.z;
				}

				screenSpacePoints.push_back(result);
				if(i == silhouette.size()-1)
				{
					if(toDeviceCoordinates(result,m_strandVertices[section][silhouette[i].second],m_camera))
					{
						if(result.z>zMax)
						{
							zMax = result.z;
						}
						if(result.z<zMin)
						{
							zMin = result.z;
						}
						screenSpacePoints.push_back(result);
					}
				}
			}
		}

		if(screenSpacePoints.size()>0)
		{
			//wrap around
			screenSpacePoints.push_back(screenSpacePoints[0]);
			std::vector<Ogre::Vector3> points;
			for(int i = 0 ; i < screenSpacePoints.size() ; i++)
			{
				Ogre::Vector3 edge1,edge2,rib,norm;
				if(i == 0)
				{
					edge1 = screenSpacePoints[1]-screenSpacePoints[0];
					rib = Ogre::Vector3(edge1.y,-edge1.x,0);
					rib.normalise();
					norm = rib;

				}
				else if(i == screenSpacePoints.size()-1)
				{
					edge1 = screenSpacePoints[i]-screenSpacePoints[i-1];
					rib = Ogre::Vector3(edge1.y,-edge1.x,0);
					rib.normalise();
					norm = rib;
				}
				else
				{
					edge1 = screenSpacePoints[i]-screenSpacePoints[i-1];
					edge2 = screenSpacePoints[i+1]-screenSpacePoints[i];
					float temp;
					temp = edge1.x;
					edge1.x = edge1.y;
					edge1.y = -temp;
					edge1.z = 0;
					edge1.normalise();
					norm = edge1;

					temp = edge2.x;
					edge2.x = edge2.y;
					edge2.y = -temp;
					edge2.z = 0;
					edge2.normalise();

					rib = edge1+edge2;
					rib.normalise();
				}
				

				//scale rib base on angle to stop corners being squeezed
				float scale = abs(rib.length()/(rib.dotProduct(norm)));

				if(scale>2)
				{
					scale = 2;
				}

				//vertex depth scale factor from artistic-sils-300dpi.pdf
				float scaleFactor = 0.5f;
				float left = 0.0f;
				float right = 1.0f+scaleFactor*((zMax+zMin-2*screenSpacePoints[i].z)/(zMax-zMin));

				scale*= std::max(left,right);

				//apply depth cue scale
				//scale*= m_fd; //this has been commented out at the moment as it seems sensitive to translation

				//multiply the scale by some value as otherwise 0 to 1 is far too big
				scale*= 0.02f;

				rib *= scale;

				points.push_back(screenSpacePoints[i]+rib);
				points.push_back(screenSpacePoints[i]-rib);
			}

			for(int i = 0 ; i < points.size()-1 ; i+= 2)
			{
				m_edgeMesh->position(points[i]);
				m_edgeMesh->textureCoord(i/2,1);
				m_edgeMesh->colour(m_idColours[section]);

				m_edgeMesh->position(points[i+1]);
				m_edgeMesh->textureCoord(i/2,0);
				m_edgeMesh->colour(m_idColours[section]);
			}
		}
		else
		{
			//if we don't give texture coordinates - it breaks the texture coordinates of any strands using this section later
			m_edgeMesh->position(Ogre::Vector3(0,0,0));
			m_edgeMesh->textureCoord(0,1);
			m_edgeMesh->colour(m_idColours[section]);
			m_edgeMesh->position(Ogre::Vector3(0,0,0));
			m_edgeMesh->textureCoord(0,0);
			m_edgeMesh->colour(m_idColours[section]);
			m_edgeMesh->position(Ogre::Vector3(0,0,0));
			m_edgeMesh->textureCoord(1,1);
			m_edgeMesh->colour(m_idColours[section]);
			m_edgeMesh->position(Ogre::Vector3(0,0,0));
			m_edgeMesh->textureCoord(0,1);
			m_edgeMesh->colour(m_idColours[section]);
		}

		m_edgeMesh->end();
	}
}
//http://www.ogre3d.org/tikiwiki/tiki-index.php?page=GetScreenspaceCoords
bool HairModel::toDeviceCoordinates(Ogre::Vector3 &result, Ogre::Vector3 &point,Ogre::Camera *camera)
{
	result = camera->getProjectionMatrix() * camera->getViewMatrix() * point;
	Ogre::Plane cameraPlane = Ogre::Plane(Ogre::Vector3(camera->getDerivedOrientation().zAxis()),
		camera->getDerivedPosition());
	if(cameraPlane.getSide(point) != Ogre::Plane::NEGATIVE_SIDE)
	{
		return false;
	}
	return true;
}

void HairModel::insertSilhouette(std::pair<int,int> element, std::vector<std::pair<int,int>> &temp, std::deque<std::pair<int,int>> &silhouette)
{
	//try to insert into silhouette
	if(silhouette.empty())
	{
		silhouette.push_front(element);
	}
	else
	{
		//see if the element should go at the top
		if(!attemptInsert(element,silhouette))
		{
			//we failed - try again later
			temp.push_back(element);
		}
	}

	//see if we can fit any of the previous elements
	for(int i = temp.size()-1 ; i >= 0 ; i--)
	{
		if(attemptInsert(temp[i],silhouette))
		{
			temp.erase(temp.begin()+i);
		}
	}
}

bool HairModel::attemptInsert(std::pair<int,int> element, std::deque<std::pair<int,int>> &silhouette)
{
	if(element.second == silhouette.front().first)
	{
		silhouette.push_front(element);
		return true;
	}
	else if(element.first == silhouette.front().first)
	{
		int x = element.first;
		element.first = element.second;
		element.second = x;
		silhouette.push_front(element);
		return true;
	}
	else if(element.first == silhouette.back().second)
	{
		silhouette.push_back(element);
		return true;
	}
	else if(element.second == silhouette.back().second)
	{
		int x = element.first;
		element.first = element.second;
		element.second = x;
		silhouette.push_back(element);
		return true;
	}
	return false;
}

//cutcode
//p755-kalins.pdf
	////we randomly sample some edges until we hit part of the silhouette
	////we then follow the edge loop until we hit the same edge again
	////we will have then identified the silhouette
	//std::vector<std::pair<int,int>> silhouette;
	//std::unordered_map<std::pair<int,int>,Edge,HashFunction,EqualFunction>::iterator it;
	
	//Edge *initialEdge;
	//bool silhouetteFound = false;
	//while(!silhouetteFound)
	//{
	//	it = m_edgeMap.begin();
	//	std::advance(it,rand() % m_edgeMap.size());
	//	Edge *edge = &(it->second);
	//	if(edge->edgeCount == 2)
	//	{
	//		if(isSilhouette(edge,section,eyeVector))
	//		{
	//			silhouette.push_back(it->first);
	//			initialEdge = edge;
	//			silhouetteFound = true;
	//		}
	//	}
	//}

	////we now have the first part of the silhouette
	//Edge *edge = initialEdge;
	//bool loopComplete = false;
	//while(!loopComplete)
	//{
	//	//iterate through linked edges looking for silhouette
	//	for(int index = 0 ; index < edge->linkedEdges.size() ; index++)
	//	{
	//		std::pair<int,int> pair = edge->linkedEdges[index];
	//		Edge *temp = &(m_edgeMap[pair]);
	//		if(temp == initialEdge)
	//		{
	//			loopComplete = true;
	//			break;
	//		}

	//		bool goingBackwards = false;
	//		//make sure we don't roll back
	//		for(int i = 0 ; i < silhouette.size() ; i++)
	//		{
	//			if(pair.first == silhouette[i].first
	//				&& pair.second == silhouette[i].second)
	//			{
	//				goingBackwards = true;
	//				break;
	//			}
	//		}

	//		if(goingBackwards)
	//		{
	//			continue;
	//		}

	//		//check edge
	//		if(isSilhouette(temp,section,eyeVector))
	//		{
	//			silhouette.push_back(pair);
	//			edge = temp;
	//			break;
	//		}
	//	}
	//}

//http://gv2.cs.tcd.ie/mcdonner/Teaching/Lectures/ComputerGraphics2012_L2.pdf
Ogre::Vector3 HairModel::calculateNormal(Ogre::Vector3 v1, Ogre::Vector3 v2, Ogre::Vector3 v3)
{
	//have to flip the order of the normal as the vertex winding appears to have gone wrong
	Ogre::Vector3 u1 = (v2-v1).normalisedCopy();
	Ogre::Vector3 u2 = (v3-v1).normalisedCopy();
	Ogre::Vector3 normal = ((u1).crossProduct(u2)).normalisedCopy();
	return normal;
}

void HairModel::createOrUpdateManualObject(bool update)
{
	//generate indices
	if(!update)
	{
		generateIndices();
		generateEdgeMap();
		generateIdColours();
	}

	//iterate through each section (one for each strand) of the hair mesh
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		if(update)
		{
			m_hairMesh->beginUpdate(section);
			m_normalMesh->beginUpdate(section);
		}
		else
		{
			m_hairMesh->begin("IETCartoonHair/HairMaterial",Ogre::RenderOperation::OT_TRIANGLE_LIST);
			m_normalMesh->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_LIST);

			m_strandVertices.push_back(std::vector<Ogre::Vector3>());
			m_strandNormals.push_back(std::vector<Ogre::Vector3>());
			m_hairSplines.push_back(Ogre::SimpleSpline());
		}

		//generate geometry
		generateVertices(update,section);
		generateNormals(update,section);

		
		//generate manual object
		//vertices + normals
		for(int vert = 0 ; vert < m_strandVertices[section].size() ; vert++)
		{
			//m_hairMesh->colour(1.0,0,0);
			m_hairMesh->position(m_strandVertices[section][vert]);
			m_hairMesh->normal(m_strandNormals[section][vert]);
			m_hairMesh->colour(m_idColours[section]);

			m_normalMesh->position(m_strandVertices[section][vert]);
			m_normalMesh->position(m_strandVertices[section][vert]+m_strandNormals[section][vert]);
		}

		//indices
		for(int index = 0 ; index < m_strandIndices.size() ; index+=3)
		{
			m_hairMesh->triangle(
				m_strandIndices[index],
				m_strandIndices[index+1],
				m_strandIndices[index+2]);
		}
		
		m_hairMesh->end();
		m_normalMesh->end();
	}
}

void HairModel::setCurveValues(float a, float b, float c)
{
	m_a = a;
	m_b = b;
	m_c = c;
}

float HairModel::determineScale(float x)
{
	//this uses the technique from A Stylized Cartoon Hair Renderer (a64-shin.pdf)
	//return Ogre::Math::Sin(((x+SHIFT_X)*Ogre::Math::PI)/(1+SHIFT_X));

	//http://www.mathopenref.com/quadraticexplorer.html

	//float func = -1.4*x*x + 0.2*x + 1.1;
	//float func = -4.6*x*x + 2.6*x + 1.1;
	//float func = -5.8*x*x + 3*x + 2.7;
	//float func = -13.9*x*x+4.9*x+6.4;
	float func = m_a*x*x + m_b*x + m_c;

	return Ogre::Math::Abs(func);
}

Ogre::Quaternion HairModel::determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1)
{
	Ogre::Vector3 dir = node1 - node0;
	return up.getRotationTo(dir);
}
