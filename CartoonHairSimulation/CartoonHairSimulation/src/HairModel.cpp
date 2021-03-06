#include "stdafx.h"
#include "HairModel.h"
#include "tinyxml2.h"
#include "Constants.h"

/*
convenience functions for converting between ogre and bullet variables
*/
Ogre::Vector3 bulletToOgreVector(btVector3 &vector)
{
	return Ogre::Vector3(vector.x(),vector.y(),vector.z());
}
btVector3 ogreToBulletVector(Ogre::Vector3 &vector)
{
	return btVector3(vector.x,vector.y,vector.z);
}
Ogre::Quaternion bulletToOgreQuaternion(btQuaternion &quaternion)
{
	return Ogre::Quaternion(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
}
btQuaternion ogreToBulletQuaternion(Ogre::Quaternion &quaternion)
{
	return btQuaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w);
}



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

HairModel::HairModel(std::string directory, std::string animation, Ogre::Camera *camera, Ogre::Light *light, Ogre::RenderWindow *window, Ogre::SceneManager *sceneMgr,
		btSoftRigidDynamicsWorld *world, float hairA,float hairB, float hairC, float blendingA, float blendingB, float blendingC,
		float edgeStiffness, float bendingStiffness, float torsionStiffness, float anchorStiffness)
{

	m_blinnSpecularEnabled = false;
	m_specularTextureEnabled = false;
	m_backlightingTextureEnabled = false;
	m_depthDetailAxisEnabled = false;
	m_variableSilhouetteIntensity = false;
	m_hatchingEnabled = false;
	m_simpleHatching = false;

	m_zMin = ZMIN;
	m_zScale = R;
	m_blinnS = BLINN_S;
	m_specTexS = SPECULAR_TEXTURE_S;
	m_backlightS = BACKLIGHTING_TEXTURE_S;
	m_strokeScale = HATCHING_STROKE_SCALE;
	m_hairResolution = NUM_HAIR_SAMPLES;
	m_shapeResolution = NUM_HAIR_SHAPE_SAMPLES;
	m_silhouetteStrokeLimit = STROKE_LIMIT;
	m_silhouetteStrokeScale = STROKE_SCALE;
	m_silhouetteA = SILHOUETTE_A;
	m_silhouetteB = SILHOUETTE_B;
	m_silhouetteC = SILHOUETTE_C;

	m_hairColour = Ogre::Vector3(RED,GREEN,BLUE);

	m_currentId = Ogre::ColourValue(0.01,0.0,0.0,1.0);

	m_camera = camera;
	m_light = light;
	m_a = hairA;
	m_b = hairB;
	m_c = hairC;
	m_blendingA = blendingA;
	m_blendingB = blendingB;
	m_blendingC = blendingC;
	m_world = world;

	m_edgeMaterial = new btSoftBody::Material();
	m_edgeMaterial->m_kLST = edgeStiffness;

	m_bendingMaterial = new btSoftBody::Material();
	m_bendingMaterial->m_kLST = bendingStiffness;

	m_torsionMaterial = new btSoftBody::Material();
	m_torsionMaterial->m_kLST = torsionStiffness;

	m_translationOffset = Ogre::Vector3(0,0,0);
	m_orientationOffset = Ogre::Quaternion(0,0,0,1);

	m_animationTime = 0;

	m_positiveInterpolation = true;

	//create render texture for storing colour id buffer for silhouette visibility testing
	//http://www.ogre3d.org/tikiwiki/Intermediate+Tutorial+7#Creating_the_render_textures
	Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual("idBuffer",
		Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,
		window->getWidth(),window->getHeight(),0,Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

	m_idBuffer = texture->getBuffer()->getRenderTarget();
	m_idBuffer->addViewport(m_camera);
	m_idBuffer->getViewport(0)->setClearEveryFrame(true);
	m_idBuffer->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
	m_idBuffer->getViewport(0)->setOverlaysEnabled(false);
	m_idBuffer->setAutoUpdated(true);

	std::string hairFrame = loadAnchorPoints(directory,animation);
	generateAnchorBody(world,world->getWorldInfo(), m_anchorPoints);
	generateHairStrands(directory+hairFrame,world,m_edgeMaterial,m_bendingMaterial,m_torsionMaterial,anchorStiffness);
	generateHairMesh(sceneMgr);
}

HairModel::~HairModel()
{
	m_hairShape.clear();
	m_strandVertices.clear();
	m_strandNormals.clear();
	m_strandIndices.clear();
	m_strandTextureCoordinates.clear();
	m_edgeMap.clear();
	m_idColours.clear();

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

	for(int i = 0 ; i < m_blendingSpringMaterials.size() ; i++)
	{
		delete m_blendingSpringMaterials[i];
	}
	m_blendingSpringMaterials.clear();

	delete m_edgeMaterial;
	delete m_bendingMaterial;
	delete m_torsionMaterial;

	m_anchorPoints.clear();
	m_anchorSplines.clear();

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

Ogre::ManualObject* HairModel::getHighlightManualObject()
{
	return m_highlightMesh;
}

Ogre::ManualObject* HairModel::getDebugEdgesManualObject()
{
	return m_debugEdges;
}

Ogre::RenderTexture* HairModel::getIdBufferTexture()
{
	return m_idBuffer;
}

void HairModel::applyHeadTransform(bool first, Ogre::Vector3 translation, Ogre::Quaternion rotation)
{
	m_translationOffset = translation;
	m_orientationOffset = rotation;
	Ogre::Vector3 offset = translation;
	btVector3 bOffset = ogreToBulletVector(offset);
	//apply translation offset to roots (and particles if first time) anchors are updated later
	//roots
	for(int strand = 0 ; strand < m_strandSoftBodies.size() ; strand++)
	{
		{
			Ogre::Vector3 pos(m_rootPoints[strand].x(),m_rootPoints[strand].y(),m_rootPoints[strand].z());
			pos = m_orientationOffset*pos;
			m_strandSoftBodies[strand]->m_nodes[0].m_x = ogreToBulletVector(pos) + bOffset;
		}

		if(first)
		{
			for(int node = 1 ; node < m_strandSoftBodies[strand]->m_nodes.size() ; node++)
			{
				btVector3 bPos = m_strandSoftBodies[strand]->m_nodes[node].m_x;
				Ogre::Vector3 pos = bulletToOgreVector(bPos);
				pos = m_orientationOffset*pos;

				m_strandSoftBodies[strand]->m_nodes[node].m_x = ogreToBulletVector(pos) + bOffset;
			}
			for(int node = 0 ; node < m_ghostStrandSoftBodies[strand]->m_nodes.size() ; node++)
			{
				btVector3 bPos = m_ghostStrandSoftBodies[strand]->m_nodes[node].m_x;
				Ogre::Vector3 pos = bulletToOgreVector(bPos);
				pos = m_orientationOffset*pos;

				m_ghostStrandSoftBodies[strand]->m_nodes[node].m_x = ogreToBulletVector(pos) + bOffset;
			}
		}
		
	}
}

void HairModel::updateManualObject()
{
	createOrUpdateManualObject(true);
	generateEdges(true);
#ifdef STYLISED_SPECULAR
	generateSpecularHighlights(true);
#endif
}

float HairModel::getSimulationScale()
{
	return m_simulationScale;
}

void HairModel::updateCollisionMargins(float min,float max)
{
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		btSoftBody *strand = m_strandSoftBodies[section];
		strand->getCollisionShape()->setMargin(Ogre::Math::RangeRandom(min,max));
	}
}

void HairModel::updateAnchors(float timestep)
{
	float increment = timestep*m_animationSpeed;
	m_animationTime += (m_positiveInterpolation)?increment:-increment;
	if(m_animationTime > 1.0f)
	{
		m_animationTime = 1.0f;
		m_positiveInterpolation = false;
	}
	if(m_animationTime < 0.0f)
	{
		m_animationTime = 0.0f;
		m_positiveInterpolation = true;
	}

	for(int anchor = 0 ; anchor < m_anchorSplines.size() ; anchor++)
	{
		Ogre::Vector3 pos = m_orientationOffset*m_anchorSplines[anchor].interpolate(m_animationTime) + m_translationOffset;
		btVector3 bPos = ogreToBulletVector(pos);
		m_anchors->m_nodes[anchor].m_x = bPos;
	}
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

	btAlignedObjectArray<btAlignedObjectArray<btVector3>> points;

	for(hair; hair ; hair = hair->NextSiblingElement())
	{
		//load vertices
		std::string name(hair->Attribute("name"));
		points.push_back(loadAnchorPositions(directory+name));
	}

	//save the first set of position so that we can create their initial positions in the physics engine
	for(int anchor = 0 ; anchor < points[0].size() ; anchor++)
	{
		m_anchorPoints.push_back(points[0][anchor]);
	}

	//create anchor point splines
	int numAnchors = points[0].size();
	for(int anchor = 0 ; anchor < numAnchors ; anchor++)
	{
		Ogre::SimpleSpline spline;
		for(int frame = 0 ; frame < points.size() ; frame++)
		{
			btVector3 bPos = points[frame][anchor];
			Ogre::Vector3 pos = bulletToOgreVector(bPos);
			spline.addPoint(pos);
		}
		spline.recalcTangents();
		m_anchorSplines.push_back(spline);
	}

	//clean up
	for(int i = 0 ; i < points.size() ; i++)
	{
		points[i].clear();
	}
	points.clear();

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
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		for(particle ; particle ; particle = particle->NextSiblingElement())
		{
			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			btVector3 point(x,y,z);

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


void HairModel::generateQuadStrip(std::vector<Ogre::Vector3> &screenSpacePoints,std::vector<Ogre::Vector3> &points, float &zMin, float &zMax, float scaleFactor)
{
	for(int i = 0 ; i < screenSpacePoints.size() ; i++)
	{
		Ogre::Vector3 edge1,edge2,rib,norm;
		//if first segment - we need to use the next rather than last point to calculate the rib vectors
		if(i == 0)
		{
			edge1 = screenSpacePoints[1]-screenSpacePoints[0];
			rib = Ogre::Vector3(edge1.y,-edge1.x,0);
			rib.normalise();
			norm = rib;
		}
		//if the last segment
		else if(i == screenSpacePoints.size()-1)
		{
			edge1 = screenSpacePoints[i]-screenSpacePoints[i-1];
			rib = Ogre::Vector3(edge1.y,-edge1.x,0);
			rib.normalise();
			norm = rib;
		}
		//if all other segments
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
		float scale = 1.0f;
		if(m_angleScalingEnabled)
		{
			scale = abs(rib.length()/(rib.dotProduct(norm)));
		}

		//vertex depth scale factor from artistic-sils-300dpi.pdf
		if(m_depthScalingEnabled)
		{
			float scaleF = 0.5f;
			float left = 0.0f;
			float right = 1.0f+scaleF*((zMax+zMin-2*screenSpacePoints[i].z)/(zMax-zMin));
			scale*= std::max(left,right);
		}

		if(scale>m_silhouetteStrokeLimit)
		{
			scale = m_silhouetteStrokeLimit;
		}

		//multiply the scale by some value as otherwise 0 to 1 is far too big
		scale*= scaleFactor;

		rib *= scale;
		points.push_back(toWorldCoordinates(screenSpacePoints[i]-rib,m_camera));
		points.push_back(toWorldCoordinates(screenSpacePoints[i]+rib,m_camera));
	}
}


void HairModel::generateHairStrands(std::string filename,btSoftRigidDynamicsWorld *world,
		btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial,btSoftBody::Material *torsionMaterial,float anchorStiffness)
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

		std::vector<float> masses;
		tinyxml2::XMLElement *particle = strand->FirstChildElement();

		for(particle ; particle ; particle = particle->NextSiblingElement())
		{

			float x = particle->FloatAttribute("x");
			float y = particle->FloatAttribute("y");
			float z = particle->FloatAttribute("z");

			btVector3 point(x,y,z);

			particles.push_back(point);
			masses.push_back(1.0f);
		}

		//generate blending springs
		if(strandCount == 0)
		{
			for(int i = 0 ; i < particles.size() ; i++)
			{
				float x = (float)i/(particles.size()-1);
				btSoftBody::Material *blendingSpring = new btSoftBody::Material();
				blendingSpring->m_kLST = anchorStiffness*abs(
					m_blendingA*x*x +
					m_blendingB*x +
					m_blendingC);
				m_blendingSpringMaterials.push_back(blendingSpring);
			}
		}

		//get the root position
		m_rootPoints.push_back(particles[0]);

		//now to create the strand softbody and its ghost nodes
		btSoftBody *hairStrand = createHairStrand(strandCount,world,particles,masses,world->getWorldInfo(),edgeMaterial,bendingMaterial,torsionMaterial);
		world->addSoftBody(hairStrand,HAIR_GROUP, BODY_GROUP | HAIR_GROUP);
		m_strandSoftBodies.push_back(hairStrand);

		btSoftBody *ghostStrand = createAndLinkGhostStrand(hairStrand,edgeMaterial,bendingMaterial,torsionMaterial);
		world->addSoftBody(ghostStrand,GHOST_GROUP,NULL);
		m_ghostStrandSoftBodies.push_back(ghostStrand);

		//clean up
		particles.clear();
		masses.clear();

		strandCount++;
	}
}

void HairModel::generateHairMesh(Ogre::SceneManager *sceneMgr)
{
	//create manual hair object
	m_hairMesh = sceneMgr->createManualObject("hair");
	m_hairMesh->setDynamic(true);

	m_normalMesh = sceneMgr->createManualObject("normals");
	m_normalMesh->setDynamic(true);

	m_edgeMesh = sceneMgr->createManualObject("edges");
	m_edgeMesh->setDynamic(true);

	m_highlightMesh = sceneMgr->createManualObject("highlight");
	m_highlightMesh->setDynamic(true);

	m_debugEdges = sceneMgr->createManualObject("debugEdges");
	m_debugEdges->setDynamic(true);

	generateHairShape();
	createOrUpdateManualObject(false);
	generateEdges(false);
#ifdef STYLISED_SPECULAR
	generateSpecularHighlights(false);
#endif
}

//based upon lines 508 to 536 of btSoftBodyHelpers.cpp
btSoftBody* HairModel::createHairStrand(int strandIndex, btSoftRigidDynamicsWorld *world, btAlignedObjectArray<btVector3> &particles, std::vector<float> &masses, btSoftBodyWorldInfo &worldInfo,
	btSoftBody::Material *edgeMaterial,btSoftBody::Material *bendingMaterial, btSoftBody::Material *torsionMaterial)
{
	//create softbody
	btSoftBody *strand = new btSoftBody(&worldInfo,particles.size(),&particles[0],&masses[0]);

#ifdef ANCHOR_SPRINGS
	//attach anchor points
	for(int node = 0 ; node < particles.size() ; node++)
	{
		strand->appendLink(&(strand->m_nodes[node]),&(m_anchors->m_nodes[strandIndex*particles.size()+node]),m_blendingSpringMaterials[node]);
	}
#endif

#ifdef EDGE_SPRINGS
	//create edge links
	for(int node = 1 ; node < particles.size() ; node++)
	{
		strand->appendLink(node-1,node,edgeMaterial);
	}
#endif

#ifdef BENDING_SPRINGS
	//create bending springs
	for(int node = 0 ; node < strand->m_nodes.size()-2 ; node++)
	{
		strand->appendLink(node,node+2,bendingMaterial);
	}
#endif

	//http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7465

	//assuming each strand is representing 100 strands or 1 gram
	strand->setTotalMass(0.001);

	//fix root
	strand->setMass(0,0);

	return strand;
}

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

#ifdef GHOST_STRAND
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
#endif

	//clean up
	particles.clear();
	masses.clear();

	ghostStrand->setTotalMass(strand->getTotalMass());

	return ghostStrand;
}

void HairModel::generateIndices()
{
	int numNodes = m_hairResolution;
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
			m_strandTextureCoordinates.push_back(Ogre::Vector2(0,1));

			m_strandIndices.push_back(index2);
			m_strandTextureCoordinates.push_back(Ogre::Vector2(1,1));

			m_strandIndices.push_back(index3);
			m_strandTextureCoordinates.push_back(Ogre::Vector2(0,0));

			//triangle 2
			m_strandIndices.push_back(index4);
			m_strandTextureCoordinates.push_back(Ogre::Vector2(1,0));

			m_strandIndices.push_back(index3);
			m_strandTextureCoordinates.push_back(Ogre::Vector2(0,0));

			m_strandIndices.push_back(index2);
			m_strandTextureCoordinates.push_back(Ogre::Vector2(1,1));
		}
	}

	//for the top
	int topIndex = (numNodes-1)*m_hairShape.size();
	for(int vert = 0 ; vert < m_hairShape.size() ; vert++)
	{
		int index1 = vert;
		int index2 = (index1==m_hairShape.size()-1)?0:vert+1;
		m_strandIndices.push_back(topIndex);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(0,0));

		m_strandIndices.push_back(index2);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(0,1));

		m_strandIndices.push_back(index1);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(1,1));
	}
	//for the tip - the tips texture coordinates are multiplied by 2 to give them the correct scale in relation to the other hair segments
	int tipIndex = topIndex+1;
	for(int vert = 0 ; vert < m_hairShape.size() ; vert++)
	{
		int offset = (numNodes-2)*m_hairShape.size();
		int index1 = vert;
		int index2 = (index1==m_hairShape.size()-1)?0:vert+1;
		m_strandIndices.push_back(tipIndex);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(0,0));

		m_strandIndices.push_back(index1+offset);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(0,2));

		m_strandIndices.push_back(index2+offset);
		m_strandTextureCoordinates.push_back(Ogre::Vector2(2,2));
	}
}

void HairModel::generateVertices(bool update, int section)
{
	btSoftBody* body = m_strandSoftBodies[section];

	//create hair shape copy - so that we can rotation it as much as we want without having to worrying about resetting it
	std::vector<Ogre::Vector3> hairShapeCopy;
	hairShapeCopy.reserve(m_hairShape.size());

	for(int i = 0 ; i < m_hairShape.size() ; i++)
	{
		hairShapeCopy.push_back(m_hairShape[i]);
	}

	//update spline
	for(int node = 0 ; node < body->m_nodes.size() ; node++)
	{
		btVector3 pos = body->m_nodes[node].m_x;
		Ogre::Vector3 position = bulletToOgreVector(pos);
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
	Ogre::Vector3 shapeDir(0,1,0);
	Ogre::Quaternion rot(0,0,0,1);
	float scale;

	float increment = 1.0f/m_hairResolution;

	for(int node = 0 ; node < m_hairResolution-1 ; node++)
	{
		float t = node*increment;
		scale = determineScale(t);

		Ogre::Vector3 nodei = m_hairSplines[section].interpolate(t);
		Ogre::Vector3 nodei_1 = m_hairSplines[section].interpolate(t+increment);

		shapeDir = rot*shapeDir;
		rot = determineRotation(shapeDir,nodei,nodei_1);

		//generate shape
		//if usual section of hair
		for(int i = 0 ; i < hairShapeCopy.size() ; i++)
		{
			hairShapeCopy[i] = rot*hairShapeCopy[i];
			Ogre::Vector3 vert = hairShapeCopy[i];
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
		int index = (m_hairResolution-1)*m_hairShape.size();
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

	m_tempEdgeMap.clear();
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

//based on A Stylized Cartoon Hair Renderer
//by Jung Shin, Michael Haller and R. Mukundan
void HairModel::generateSpecularHighlights(bool update)
{
	if (!update)
	{
		//generate a number of sections
		for(int i = 0 ; i < MAX_GROUP_SECTIONS ; i++)
		{
			m_highlightMesh->begin("IETCartoonHair/HighlightMaterial",Ogre::RenderOperation::OT_TRIANGLE_STRIP);
			//if we don't give texture coordinates - it breaks the texture coordinates of any strands using this section later
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,1);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,0);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(1,1);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,1);
			m_highlightMesh->end();
		}
	}

	//find candidates
	std::vector<Ogre::Vector3> candidates;
	candidates.reserve(CANDIDATES_RESERVE_SIZE);
	for(int body = 0 ; body < m_strandSoftBodies.size() ; body++)
	{
		btSoftBody *strand = m_strandSoftBodies[body];
		Ogre::Vector3 point = Ogre::Vector3::ZERO;
		Ogre::Vector3 lastPoint = Ogre::Vector3::ZERO;
		for(int node = 0 ; node < strand->m_nodes.size() ; node++)
		{
			//limit how much of the strand we use
			float pos = float(node)/(strand->m_nodes.size()-1);
			if(pos>TIP_EXCLUDE)
			{
				break;
			}

			lastPoint = point;
			Ogre::Vector3 point = bulletToOgreVector(strand->m_nodes[node].m_x);

			//calculate H vector
			Ogre::Vector3 h = point-m_translationOffset;
			h.normalise();

			//calculate tangent vector
			Ogre::Vector3 tangent;
			//if we are at the first particle we will just use H as the tangent
			if(node == 0)
			{
				tangent = h;
			}
			else
			{
				tangent = lastPoint-point;
				tangent.normalise();
			}

			//calculate right vector
			Ogre::Vector3 right = tangent.crossProduct(h);
			right.normalise();

			//calculate normal
			Ogre::Vector3 normal = tangent.crossProduct(right);
			normal.normalise();

			//calculate weight vector
			Ogre::Vector3 weightVector = normal*SPECULAR_WEIGHT+tangent*(1-SPECULAR_WEIGHT);

			//calculate light vectore
			Ogre::Vector3 lightDirection = m_light->getPosition()-point;

			//specular value
			float specular = KS*Ogre::Math::Pow(glm::max(lightDirection.dotProduct(weightVector),0.0f),SHININESS);

			//determine if a candidate or not
			if(specular >= SPECULAR_THRESHOLD)
			{
				//m_highlightMesh->position(point);
				candidates.push_back(point);
			}
		}
	}

	//time to create groups
	std::vector<std::vector<Ogre::Vector3>> groups;

	while(!candidates.empty())
	{
		std::vector<Ogre::Vector3> group;
		int index = candidates.size()-1;
		group.push_back(candidates[index]);

		//remove from candidates
		candidates.erase(candidates.begin()+(candidates.size()-1));

		for(int i = candidates.size()-1 ; i >= 0 ; i--)
		{
			bool accepted = false;
			for(int j = 0 ; j < group.size() ; j++)
			{
				float length = (candidates[i]-group[j]).length();
				if(length >= MIN_MERGING_DISTANCE && length <= MAX_MERGING_DISTANCE)
				{
					accepted = true;
					break;
				}
			}
			if(group.size() >= MAX_GROUP_SIZE)
			{
				break;
			}
			if(accepted)
			{
				group.push_back(candidates[i]);
				candidates.erase(candidates.begin()+i);
			}
		}
		if(group.size() >= MIN_GROUP_SIZE)
		{
			groups.push_back(group);
		}
	}

	//determin min and max x,y and z
	std::vector<Ogre::Vector3> maxes;
	std::vector<Ogre::Vector3> mins;
	for(int group = 0 ; group < groups.size() ; group++)
	{
		Ogre::Vector3 min,max;
		min.x = min.y = min.z = std::numeric_limits<float>::max();
		max.x = max.y = max.z = std::numeric_limits<float>::min();
		for(int p = 0 ; p < groups[group].size() ; p++)
		{
			//find min and maxes
			if(groups[group][p].x < min.x)
				min.x = groups[group][p].x;
			if(groups[group][p].y < min.y)
				min.y = groups[group][p].y;
			if(groups[group][p].z < min.z)
				min.z = groups[group][p].z;

			if(groups[group][p].x > max.x)
				max.x = groups[group][p].x;
			if(groups[group][p].y > max.y)
				max.y = groups[group][p].y;
			if(groups[group][p].z > max.z)
				max.z = groups[group][p].z;
		}
	}

	for(int group = 0 ; group < groups.size() ; group++)
	{
		//we now have the accepted points - time to sort
		//find biggest range
		float xRange = Ogre::Math::Abs(maxes[group].x-mins[group].x);
		float yRange = Ogre::Math::Abs(maxes[group].y-mins[group].y);
		float zRange = Ogre::Math::Abs(maxes[group].z-mins[group].z);

		/*if(xRange > yRange && xRange > zRange)
		{
			m_candidateSort.sortType = SortType::XSORT;
		}
		else if(yRange > xRange && yRange > zRange)
		{
			m_candidateSort.sortType = SortType::YSORT;
		}
		else
		{
			m_candidateSort.sortType = SortType::ZSORT;
		}*/

		m_candidateSort.sortType = SortType::YSORT;

		std::sort(groups[group].begin(),groups[group].end(),m_candidateSort);

		//now to generate the strokes
		std::vector<Ogre::Vector3> screenSpacePoints;
		std::vector<Ogre::Vector3> points;
		float zMin = std::numeric_limits<float>::max();
		float zMax = std::numeric_limits<float>::min();

		Ogre::Vector3 result = toDeviceCoordinates(groups[group][0],m_camera);
		screenSpacePoints.push_back(result);

		if(result.z>zMax)
		{
			zMax = result.z;
		}
		if(result.z<zMin)
		{
			zMin = result.z;
		}

		result = toDeviceCoordinates(groups[group][groups[group].size()-1],m_camera);
		screenSpacePoints.push_back(result);

		if(result.z>zMax)
		{
			zMax = result.z;
		}
		if(result.z<zMin)
		{
			zMin = result.z;
		}

		if(group < MAX_GROUP_SECTIONS)
		{
			generateQuadStrip(screenSpacePoints,points,zMin,zMax,HIGHLIGHT_SCALE);
			m_highlightMesh->beginUpdate(group);
		
			/*
			for(int i = 0 ; i < points.size() ; i+=2)
			{
				m_highlightMesh->position(points[i]);
				m_highlightMesh->textureCoord(i/2,1);
			
				m_highlightMesh->position(points[i+1]);
				m_highlightMesh->textureCoord(i/2,0);
			}
			*/
	
			m_highlightMesh->position(points[0]);
			m_highlightMesh->textureCoord(0,1);
			
			m_highlightMesh->position(points[1]);
			m_highlightMesh->textureCoord(0,0);

			m_highlightMesh->position(points[points.size()-2]);
			m_highlightMesh->textureCoord(1,1);
			
			m_highlightMesh->position(points[points.size()-1]);
			m_highlightMesh->textureCoord(1,0);
	
			m_highlightMesh->end();
		}
	}

	if(groups.size() < MAX_GROUP_SECTIONS)
	{
		for(int i = groups.size() ; i < MAX_GROUP_SECTIONS ; i++)
		{
			m_highlightMesh->beginUpdate(i);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,1);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,0);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(1,1);
			m_highlightMesh->position(Ogre::Vector3(0,0,0));
			m_highlightMesh->textureCoord(0,1);
			m_highlightMesh->end();
		}
	}
}

void HairModel::generateEdges(bool update)
{
	Ogre::Vector3 eyeVector = m_camera->getDerivedDirection();
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		if(update)
		{
			m_edgeMesh->beginUpdate(section);
			m_debugEdges->beginUpdate(section);
		}
		else
		{
			m_edgeMesh->begin("IETCartoonHair/EdgeMaterial",Ogre::RenderOperation::OT_TRIANGLE_STRIP);
			m_debugEdges->begin("BaseWhiteNoLighting",Ogre::RenderOperation::OT_LINE_LIST);
		}

		m_edgeMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("width",(float)m_camera->getViewport()->getActualWidth());
		m_edgeMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("height",(float)m_camera->getViewport()->getActualHeight());

		std::deque<std::pair<int,int>> silhouette;
		std::vector<std::pair<int,int>> temp;
		temp.reserve(TEMP_SILHOUETTE_RESERVE_SIZE);

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
				insertSilhouette(currentPair,temp,silhouette);
			}

		}

		
		//we now have the silhouettes - now to convert the points to device coordinates
		//based off of artistic-sils-300dpi.pdf - Artistic Silhouettes a Hybrid Approach
		//by J.D. Northrup and Lee Markosian
		//and
		//http://www.ogre3d.org/tikiwiki/tiki-index.php?page=GetScreenspaceCoords
		float zMin = std::numeric_limits<float>::max();
		float zMax = std::numeric_limits<float>::min();
		std::vector<Ogre::Vector3> screenSpacePoints;
		screenSpacePoints.reserve(silhouette.size()+1);
		std::vector<float> silhouetteIntensities;
		silhouetteIntensities.reserve(silhouette.size()+1);

		for(int i = 0 ; i < silhouette.size() ; i++)
		{
			float silhouetteIntensity;
			if(m_variableSilhouetteIntensity)
			{
				btVector3 pos1 = ogreToBulletVector(m_strandVertices[section][silhouette[i].first]);
				btVector3 pos2 = ogreToBulletVector(m_strandVertices[section][silhouette[i].second]);
				btVector3 pos = (pos1+pos2)/2.0f;
				btSoftBody *strnd = m_strandSoftBodies[section];

				float minDist = std::numeric_limits<float>::max();
				float minNode = 0;
				for(int n = 0 ; n < strnd->m_nodes.size() ; n++)
				{
					float length = (strnd->m_nodes[n].m_x - pos).length();
					if(length < minDist)
					{
						minDist = length;
						minNode = n;
					}
				}

				float t = (float)minNode/(strnd->m_nodes.size()-1);
				silhouetteIntensity = Ogre::Math::Abs(m_silhouetteA*t*t + m_silhouetteB*t + m_silhouetteC);//(1.0f-Ogre::Math::Cos(t*Ogre::Math::PI))*0.5f;
				silhouetteIntensities.push_back(silhouetteIntensity);
				//since 2 points are added on the last silhouette - we need to add the last intensity again
				//or else we will have an index overflow
				if(i == silhouette.size()-1)
				{
					silhouetteIntensities.push_back(silhouetteIntensity);
				}
			}


			m_debugEdges->position(m_strandVertices[section][silhouette[i].first]);
			if(m_variableSilhouetteIntensity)
			{
				m_debugEdges->colour(0,silhouetteIntensity,0);
			}
			m_debugEdges->position(m_strandVertices[section][silhouette[i].second]);

			Ogre::Vector3 result = toDeviceCoordinates(m_strandVertices[section][silhouette[i].first],m_camera);

			if(result.z>zMax)
			{
				zMax = result.z;
			}
			if(result.z<zMin)
			{
				zMin = result.z;
			}
			screenSpacePoints.push_back(result);

			//make sure the last point is added
			if(i == silhouette.size()-1)
			{
				result = toDeviceCoordinates(m_strandVertices[section][silhouette[i].second],m_camera);

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

		if(screenSpacePoints.size()>0)
		{
			std::vector<Ogre::Vector3> points;
			points.reserve(screenSpacePoints.size()*2);
			generateQuadStrip(screenSpacePoints,points,zMin,zMax,STROKE_SCALE);

			for(int i = 0 ; i < points.size() ; i+= 2)
			{
				Ogre::ColourValue colour = m_idColours[section];
				if(m_variableSilhouetteIntensity)
				{
					colour.a = silhouetteIntensities[i/2];
				}

				m_edgeMesh->position(points[i]);
				m_edgeMesh->textureCoord(i/2,1);
				m_edgeMesh->colour(colour);
				
				m_edgeMesh->position(points[i+1]);
				m_edgeMesh->textureCoord(i/2,0);
				m_edgeMesh->colour(colour);
				
			}
		}
		else
		{
			//even though if we are here - the object is invisible - if we don't give texture coordinates - it breaks the texture coordinates of any strands using this section later
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
		m_debugEdges->end();
	}
}

/*
based on http://www.ogre3d.org/tikiwiki/tiki-index.php?page=GetScreenspaceCoords and
http://gamedev.stackexchange.com/questions/50336/problems-projecting-a-point-to-screen?rq=1
*/
Ogre::Vector3 HairModel::toDeviceCoordinates(Ogre::Vector3 &point,Ogre::Camera *camera)
{
	Ogre::Vector4 p(point.x,point.y,point.z,1);

	p = camera->getViewMatrix()*p;
	p = camera->getProjectionMatrix()*p;

	Ogre::Vector3 result(p.x,p.y,p.z);
	result /= p.w;
	return result;
}

Ogre::Vector3 HairModel::toWorldCoordinates(Ogre::Vector3 &point, Ogre::Camera *camera)
{
	Ogre::Vector3 p(point);
	p = camera->getProjectionMatrix().inverse()*p;
	p = camera->getViewMatrix().inverse()*p;
	return p;
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

//based off of http://gv2.cs.tcd.ie/mcdonner/Teaching/Lectures/ComputerGraphics2012_L2.pdf
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

		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("blinnS",m_blinnS);
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("specularTextureS",m_specTexS);
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("backlightingS",m_backlightS);
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("hairColour",m_hairColour);
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getVertexProgramParameters()->setNamedConstant("strokeScale",m_strokeScale);

		if(m_hatchingEnabled)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("hatchingEnabled",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("hatchingEnabled",0);
		}

		if(m_simpleHatching)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("simpleHatching",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("simpleHatching",0);
		}

		if(m_blinnSpecularEnabled)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("blinnEnabled",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("blinnEnabled",0);
		}
		
		if(m_specularTextureEnabled)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("specularTextureEnabled",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("specularTextureEnabled",0);
		}

		if(m_backlightingTextureEnabled)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("backlightingEnabled",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("backlightingEnabled",0);
		}

		if(m_depthDetailAxisEnabled)
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("depthAxisEnabled",1);
		}
		else
		{
			m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters()->setNamedConstant("depthAxisEnabled",0);
		}

		//send the depth values to the hair shader so that we can use it to index the y-axis of the cartoon texture
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getVertexProgramParameters()->setNamedConstant("zMax",m_zMin*m_zScale);
		m_hairMesh->getSection(section)->getMaterial()->getTechnique(0)->getPass(0)->getVertexProgramParameters()->setNamedConstant("zMin",m_zMin);

		//generate geometry
		generateVertices(update,section);
		generateNormals(update,section);
		
		//generate manual object
		//vertices + normals
		for(int vert = 0 ; vert < m_strandVertices[section].size() ; vert++)
		{
			m_normalMesh->position(m_strandVertices[section][vert]);
			m_normalMesh->position(m_strandVertices[section][vert]+m_strandNormals[section][vert]);
		}

		//indices
		for(int index = 0 ; index < m_strandIndices.size() ; index++)
		{
			m_hairMesh->position(m_strandVertices[section][m_strandIndices[index]]);
			m_hairMesh->normal(m_strandNormals[section][m_strandIndices[index]]);
			//m_hairMesh->tangent(Ogre::Vector3::UNIT_X);
			if(m_sobelEnabled)
			{
				m_hairMesh->colour(0,1,0);
			}
			else
			{
				m_hairMesh->colour(m_idColours[section]);
			}
			m_hairMesh->textureCoord(m_strandTextureCoordinates[index]);
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
	float func = m_a*x*x + m_b*x + m_c;
	return Ogre::Math::Abs(func);
}

Ogre::Quaternion HairModel::determineRotation(Ogre::Vector3 up, Ogre::Vector3 node0, Ogre::Vector3 node1)
{
	Ogre::Vector3 dir = node1 - node0;
	return up.getRotationTo(dir);
}

void HairModel::enableBlinnSpecular(bool value)
{
	m_blinnSpecularEnabled = value;
}

void HairModel::enableSpecularTexture(bool value)
{
	m_specularTextureEnabled = value;
}

void HairModel::enableBacklightingTexture(bool value)
{
	m_backlightingTextureEnabled = value;
}

void HairModel::enableDepthDetailAxis(bool value)
{
	m_depthDetailAxisEnabled = value;
}

void HairModel::enableVariableSilhouetteIntensity(bool value)
{
	m_variableSilhouetteIntensity = value;
}

void HairModel::enableSobel(bool value)
{
	m_sobelEnabled = value;
}

void HairModel::enableHatching(bool value)
{
	m_hatchingEnabled = value;
}

void HairModel::enableSimpleHatching(bool value)
{
	m_simpleHatching = value;
}

void HairModel::enableAngleScaling(bool value)
{
	m_angleScalingEnabled = value;
}

void HairModel::enableDepthScaling(bool value)
{
	m_depthScalingEnabled = value;
}

void HairModel::setSilhouetteStrokeScale(float value)
{
	m_silhouetteStrokeScale = value;
}

void HairModel::setSilhouetteStrokeLimit(float value)
{
	m_silhouetteStrokeLimit = value;
}

void HairModel::setZMin(float value)
{
	m_zMin = value;
}

void HairModel::setZScale(float value)
{
	m_zScale = value;
}

void HairModel::setBlinnS(float value)
{
	m_blinnS = value;
}
void HairModel::setSpecularTextureS(float value)
{
	m_specTexS = value;
}

void HairModel::setBacklightingS(float value)
{
	m_backlightS = value;
}

void HairModel::setHairColour(Ogre::Vector4 value)
{
	m_hairColour = value;
}

void HairModel::setStrokeScale(float value)
{
	m_strokeScale = value;
}

void HairModel::setEdgeSpringStiffness(float value)
{
	m_edgeMaterial->m_kLST = value;
	updateLinks();
}
void HairModel::setBendingSpringStiffness(float value)
{
	m_bendingMaterial->m_kLST = value;
	updateLinks();
}
void HairModel::setTorsionSpringStiffness(float value)
{
	m_torsionMaterial->m_kLST = value;
	updateLinks();
}
void HairModel::setBlendingSpringStiffness(float value)
{
	for(int spring = 0 ; spring < m_blendingSpringMaterials.size() ; spring++)
	{
		float x = (float)spring/(m_blendingSpringMaterials.size()-1);
		m_blendingSpringMaterials[spring]->m_kLST = 
			value*abs(
			m_blendingA*x*x +
			m_blendingB*x +
			m_blendingC);
	}
	updateLinks();
}
void HairModel::setBlendingCurve(float a,float b, float c)
{
	m_blendingA = a;
	m_blendingB = b;
	m_blendingC = c;
}

void HairModel::setSilhouetteCurve(float a, float b, float c)
{
	m_silhouetteA = a;
	m_silhouetteB = b;
	m_silhouetteC = c;
}

void HairModel::updateLinks()
{
	for(int strand = 0 ; strand < m_strandSoftBodies.size() ; strand++)
	{
		m_strandSoftBodies[strand]->updateLinkConstants();
	}
	for(int strand = 0 ; strand < m_ghostStrandSoftBodies.size() ; strand++)
	{
		m_ghostStrandSoftBodies[strand]->updateLinkConstants();
	}
	m_anchors->updateLinkConstants();
}

void HairModel::rebuildMesh(int hairResolution, int shapeResolution)
{
	m_hairResolution = hairResolution;
	m_shapeResolution = shapeResolution;

	//rebuild hair shape
	m_hairShape.clear();
	generateHairShape();

	//rebuild indices
	m_strandIndices.clear();
	m_strandTextureCoordinates.clear();
	generateIndices();

	//rebuild edge map
	m_edgeMap.clear();
	generateEdgeMap();

	//rebuild vertices + normals
	m_hairSplines.clear();
	for(int section = 0 ; section < m_strandSoftBodies.size() ; section++)
	{
		m_hairSplines.push_back(Ogre::SimpleSpline());
		m_strandVertices[section].clear();
		m_strandNormals[section].clear();
		generateVertices(false,section);
		generateNormals(false,section);
	}

}

void HairModel::generateHairShape()
{
	//create hair shape
	Ogre::Vector3 vertex(0,0,-0.1);
	Ogre::Radian rad(Ogre::Degree(-360.0f/m_shapeResolution));
	m_hairShape.push_back(vertex);
	for(int sample = 1 ; sample < m_shapeResolution ; sample++)
	{
		vertex = Ogre::Quaternion(rad,Ogre::Vector3::UNIT_Y)*vertex;
		m_hairShape.push_back(vertex);
	}
}