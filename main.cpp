#include <stdlib.h>
#include <time.h>
#include <btBulletDynamicsCommon.h>
#include <ogre.h>
//#include "FAASTClient.h"
#include "PhysicsInterface.h"
#include "GraphicsInterface.h";
#include <OISInputManager.h>
#include <OISKeyboard.h>
//#include "KinematicMotionState.h"
#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"
//#include "DotSceneLoader.h"
#include "GameTimer.h"
#include "AnimationManager.h"
#include "OpenALSoundSystem.h"
#include "tinyxml.h"
#include "OgreTextAreaOverlayElement.h"
#include "OgreFontManager.h"
#include <OgreOverlayManager.h>
#include "ConvexDecomposition\ConvexDecomposition.h"
#include "ConvexDecomposition\cd_wavefront.h"

#pragma region Miscellaneous Variables
const int SOUND_FADE = 300;
const int MOVEMENT_SPEED = 100000;
const int IMPACT_FORCE = 10000;
const float FIRE_WAIT_TIME = 250; //0.25 second
float fireTimer = FIRE_WAIT_TIME;
SceneNode * projectile_sn = NULL;
int projectileNum = 0;
#pragma endregion

#pragma region OpenAL Variables
OpenALSoundSystem sound;
double soundTimer = 0;
double soundDuration = 0;
int soundVar = 0;
bool soundBool = false;

double zeroVel[3] = {0, 0, 0};
double playerPos[3] = {431.265, 0, -1106.07};
double shipEMPPos[3] = {431.265, 0, -1500};

const double PAUSE_TIME = 2;

string SoundString[] = {"Sounds/clank.wav", "Sounds/warpin.wav", "Sounds/nostromo.wav", "Sounds/empblast.wav", "Sounds/depressurization.wav", "Sounds/ambience1.wav", "Sounds/ambience2.wav", "Sounds/ambience3.wav", "Sounds/ambience4.wav", "Sounds/ambience5.wav", 
						"Sounds/ambience6.wav", "Sounds/ambience7.wav", "Sounds/ambience8.wav", "Sounds/ambience9.wav", "Sounds/cinematicboom.wav", "Sounds/scarysignal1.wav", "Sounds/scarysignal2.wav", "Sounds/silence.wav"};

int SoundTimer[] = {1, 11, 14, 3, 6, 33, 42, 43, 40, 23, 57, 44, 38, 27, 7, 42, 8, 10};
const int RANDOMSTARTINT = 5;
const int SILENCEINT = 17;
#pragma endregion

#pragma region Objects Lists
std::list<btRigidBody*> PhysicsObjects;
std::list<SceneNode*> OgreObjects;
#pragma endregion

#pragma region Ogre + Bullet Variables
Ogre::Root *root;
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody *player;

btScalar scale = 8;
btVector3 dimensions(.264 * scale, .363 * scale, .530 * scale); //Taken from blender
#pragma endregion

#pragma region Collision Variables
btVector3	worldAabbMin(-40000,-40000,-40000);
btVector3	worldAabbMax(40000,40000,40000);

btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

//btdynamicsWorld*	dynamicsWorld = new btdynamicsWorld(dispatcher,broadphase,collisionConfiguration);
#pragma endregion

#pragma region Cargo Variables
bool cargoBool[6] = {false};
#pragma endregion

void mainPlaySound(double elapsedTime)
{
	switch(soundVar)
	{
		case 0:
			soundTimer += elapsedTime;
			if(soundTimer < SoundTimer[3])
			{
				if(!soundBool)
				{
					sound.createSource(SoundString[3], shipEMPPos, zeroVel);
					sound.assignSourceSound(SoundString[3], SoundString[3], 1, 1, 0);
					sound.playSound(SoundString[3]);
					soundBool = true;
				}
			}
			else
			{
				++soundVar;
				soundTimer = 0;
				soundBool = false;
			}
			break;

		case 1:
			soundTimer += elapsedTime;
			if(soundTimer >= PAUSE_TIME)
			{
				++soundVar;
				soundTimer = 0;
			}
			break;

		case 2:
			soundTimer += elapsedTime;
			if(soundTimer < SoundTimer[4])
			{
				if(!soundBool)
				{
					sound.createSource(SoundString[4], shipEMPPos, zeroVel);
					sound.assignSourceSound(SoundString[4], SoundString[4], 1, 1, 0);
					sound.playSound(SoundString[4]);
					soundBool = true;
				}
			}
			else
			{
				soundVar = RANDOMSTARTINT;
				soundTimer = 0;
				soundBool = false;
			}

			break;
	}
	//printf("soundTimer: %4.1f\n", soundTimer);
}

void playSound(double elapsedTime, int soundNum, double soundLoc[], bool updatePosition)
{
	soundTimer += elapsedTime;

	if(updatePosition)
	{
		sound.setSourceData("123", soundLoc, zeroVel);
	}
	else
	{
		soundDuration = SoundTimer[soundNum];
	}

	if(soundTimer < soundDuration && !updatePosition)
	{
		sound.createSource(SoundString[soundNum], soundLoc, zeroVel);
		sound.assignSourceSound(SoundString[soundNum], SoundString[soundNum], 1, 1, 0);
		sound.playSound(SoundString[soundNum]);
		soundBool = true;
	}
	else if(soundTimer >= soundDuration)
	{
		soundTimer = 0;
		soundBool = false;
		if(soundVar != SILENCEINT)
		{
			soundVar = SILENCEINT;
		}
		else
		{
			soundVar = RANDOMSTARTINT;
		}
	}
}

void playWarpInSound(double loc[])
{
	sound.createSource(SoundString[1], loc, zeroVel);
	sound.assignSourceSound(SoundString[1], SoundString[1], 1, 1, 0);
	sound.playSound(SoundString[1]);
}

void playNostromo(double loc[])
{
	sound.createSource(SoundString[2], loc, zeroVel);
	sound.assignSourceSound(SoundString[2], SoundString[2], 1, 1, 1);
	sound.playSound(SoundString[2]);
}

void playClankSound(double loc[])
{
	sound.createSource(SoundString[0], loc, zeroVel);
	sound.assignSourceSound(SoundString[0], SoundString[0], 1, 1, 0);
	sound.playSound(SoundString[0]);
}

void CollisionFeedback()
{
	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				//const btVector3& ptB = pt.getPositionWorldOnB();
				//const btVector3& normalOnB = pt.m_normalWorldOnB;

				//double ptLoc[3] = {ptA.getX(), ptA.getY(), ptA.getZ()};
				//playClankSound(ptLoc);

				std::list<btRigidBody*>::iterator Pit = PhysicsObjects.begin();
				for(int i = 0; i < PhysicsObjects.size(); i++, Pit++)
				{
					if((*Pit == obA || *Pit == obB) && cargoBool[i] == false)
					{
						/*
						btTransform tr;
						(*Pit)->getMotionState()->getWorldTransform(tr);

						string pSName = "Clank" + i;
						double pSLoc[] = {tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ()};
						sound.setSourceData(pSName, pSLoc, zeroVel);
						sound.playSound(pSName);
						*/

						cargoBool[i] = true;
					}
					else
					{
						cargoBool[i] = false;
					}
				}
			}
		}
	}
}

void createSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16)
 {
     MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
     SubMesh *pSphereVertex = pSphere->createSubMesh();
 
     pSphere->sharedVertexData = new VertexData();
     VertexData* vertexData = pSphere->sharedVertexData;
 
     // define the vertex format
     VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
     size_t currOffset = 0;
     // positions
     vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
     currOffset += VertexElement::getTypeSize(VET_FLOAT3);
     // normals
     vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
     currOffset += VertexElement::getTypeSize(VET_FLOAT3);
     // two dimensional texture coordinates
     vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
     currOffset += VertexElement::getTypeSize(VET_FLOAT2);
 
     // allocate the vertex buffer
     vertexData->vertexCount = (nRings + 1) * (nSegments+1);
     HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
     VertexBufferBinding* binding = vertexData->vertexBufferBinding;
     binding->setBinding(0, vBuf);
     float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));
 
     // allocate index buffer
     pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
     pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
     HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
     unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));
 
     float fDeltaRingAngle = (Math::PI / nRings);
     float fDeltaSegAngle = (2 * Math::PI / nSegments);
     unsigned short wVerticeIndex = 0 ;
 
     // Generate the group of rings for the sphere
     for( int ring = 0; ring <= nRings; ring++ ) {
         float r0 = r * sinf (ring * fDeltaRingAngle);
         float y0 = r * cosf (ring * fDeltaRingAngle);
 
         // Generate the group of segments for the current ring
         for(int seg = 0; seg <= nSegments; seg++) {
             float x0 = r0 * sinf(seg * fDeltaSegAngle);
             float z0 = r0 * cosf(seg * fDeltaSegAngle);
 
             // Add one vertex to the strip which makes up the sphere
             *pVertex++ = x0;
             *pVertex++ = y0;
             *pVertex++ = z0;
 
             Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
             *pVertex++ = vNormal.x;
             *pVertex++ = vNormal.y;
             *pVertex++ = vNormal.z;
 
             *pVertex++ = (float) seg / (float) nSegments;
             *pVertex++ = (float) ring / (float) nRings;
 
             if (ring != nRings) {
                                // each vertex (except the last) has six indices pointing to it
                 *pIndices++ = wVerticeIndex + nSegments + 1;
                 *pIndices++ = wVerticeIndex;               
                 *pIndices++ = wVerticeIndex + nSegments;
                 *pIndices++ = wVerticeIndex + nSegments + 1;
                 *pIndices++ = wVerticeIndex + 1;
                 *pIndices++ = wVerticeIndex;
                 wVerticeIndex ++;
             }
         }; // end for seg
     } // end for ring
 
     // Unlock
     vBuf->unlock();
     iBuf->unlock();
     // Generate face list
     pSphereVertex->useSharedVertices = true;
 
     // the original code was missing this line:
     pSphere->_setBounds( AxisAlignedBox( Vector3(-r, -r, -r), Vector3(r, r, r) ), false );
     pSphere->_setBoundingSphereRadius(r);
         // this line makes clear the mesh is loaded (avoids memory leaks)
         pSphere->load();
  }

void loadObj(const char* fileName, btDynamicsWorld *dynamicsWorld, btVector3 &position, btScalar scaling = 1.f)
{
	ConvexDecomposition::WavefrontObj wo;
	int loadedWO = wo.loadObj(fileName);

	if(loadedWO)
	{
		btTriangleMesh* trimesh = new btTriangleMesh();

		btVector3 localScaling(scaling, scaling, scaling);
		
		int i;
		for ( i=0;i<wo.mTriCount;i++)
		{
			int index0 = wo.mIndices[i*3];
			int index1 = wo.mIndices[i*3+1];
			int index2 = wo.mIndices[i*3+2];

			btVector3 vertex0(wo.mVertices[index0*3], wo.mVertices[index0*3+1],wo.mVertices[index0*3+2]);
			btVector3 vertex1(wo.mVertices[index1*3], wo.mVertices[index1*3+1],wo.mVertices[index1*3+2]);
			btVector3 vertex2(wo.mVertices[index2*3], wo.mVertices[index2*3+1],wo.mVertices[index2*3+2]);
			
			vertex0 *= localScaling;
			vertex1 *= localScaling;
			vertex2 *= localScaling;

			trimesh->addTriangle(vertex0,vertex1,vertex2);
		}

		
		btBvhTriangleMeshShape * tmpConvexShape = new btBvhTriangleMeshShape(trimesh, true, true);
		btDefaultMotionState *motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), position));
		btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, motionState, tmpConvexShape, btVector3(0,0,0));
		btRigidBody *rigidBody = new btRigidBody(rigidBodyCI);
		rigidBody->setFriction(btScalar(.9f));
		rigidBody->setRestitution(btScalar(.9f));
		dynamicsWorld->addRigidBody(rigidBody);
	}
}

std::list<btRigidBody*>* placeCargo(btDynamicsWorld *dynamicsWorld)
{
	std::list<btRigidBody*> *retn = new std::list<btRigidBody*>();

	btCollisionShape *boxShape = new btBoxShape(dimensions);//Vector taken from the blender object

	std::list<btVector3> Positions;
	double cVec0[] = {382.304, -10.7715, -1098.74};
	double cVec1[] = {380.113, -10.5812, -1145.91};
	double cVec2[] = {492.454, -12.7658, -1077.43};
	double cVec3[] = {494.931, -12.9534, -1038.67};
	double cVec4[] = {482.019, -12.7758, -1035.92};
	double cVec5[] = {482.938, -12.7734, -1071.23};

	Positions.push_back(btVector3(cVec0[0], cVec0[1], cVec0[2]));
	Positions.push_back(btVector3(cVec1[0], cVec1[1], cVec1[2]));
	Positions.push_back(btVector3(cVec2[0], cVec2[1], cVec2[2]));
	Positions.push_back(btVector3(cVec3[0], cVec3[1], cVec3[2]));
	Positions.push_back(btVector3(cVec4[0], cVec4[1], cVec4[2]));
	Positions.push_back(btVector3(cVec5[0], cVec5[1], cVec5[2]));

	/*
	sound.createSource("Clank0", cVec0, zeroVel);
	sound.assignSourceSound("Clank0", SoundString[0], 1, 1, 0);
	sound.createSource("Clank1", cVec1, zeroVel);
	sound.assignSourceSound("Clank1", SoundString[0], 1, 1, 0);
	sound.createSource("Clank2", cVec2, zeroVel);
	sound.assignSourceSound("Clank2", SoundString[0], 1, 1, 0);
	sound.createSource("Clank3", cVec3, zeroVel);
	sound.assignSourceSound("Clank3", SoundString[0], 1, 1, 0);
	sound.createSource("Clank4", cVec4, zeroVel);
	sound.assignSourceSound("Clank4", SoundString[0], 1, 1, 0);
	sound.createSource("Clank5", cVec5, zeroVel);
	sound.assignSourceSound("Clank5", SoundString[0], 1, 1, 0);
	*/

	std::list<btVector3>::iterator it = Positions.begin();
	//Create a physics object at each position
	for(int i = 0; i < Positions.size(); i++, it++)
	{
		btVector3 Position = (*it);

		btScalar mass = 100;
		btVector3 inertia;
		boxShape->calculateLocalInertia(mass, inertia);

		btDefaultMotionState *newPhysicsBoxMS = new btDefaultMotionState(btTransform(btQuaternion(1, 0, 0, 0), Position));
		btRigidBody::btRigidBodyConstructionInfo newPhysicsBoxCI(mass, newPhysicsBoxMS, boxShape, inertia);
		btRigidBody *newPhysicsBox = new btRigidBody(newPhysicsBoxCI);
		dynamicsWorld->addRigidBody(newPhysicsBox);
		newPhysicsBox->setRestitution(btScalar(.8f));

		retn->push_back(newPhysicsBox);
	}

	return retn;
}

void initialFailure()
{
	dynamicsWorld->setGravity(btVector3(0, 0, 0));
	std::list<btRigidBody*>::iterator it = PhysicsObjects.begin();
	for(int i = 0; i < PhysicsObjects.size(); i++, it++)
	{
		btVector3 force(0, IMPACT_FORCE, 0);
		(*it)->applyCentralForce(force);
	}

	btVector3 force(0, IMPACT_FORCE, 0);
	player->applyCentralForce(force);
}

int main(int argc, char* argv[])
{
	//Set up rand seed
	srand( (unsigned)time( NULL ) );

	//btRigidBody *rightArm;
	//btRigidBody *leftArm;

	Ogre::SceneNode *rightArmOgre;
	Ogre::SceneNode *leftArmOgre;

	//FAASTClient kinect;

	BtOgre::DebugDrawer* dbgdraw;

	//Set up OpenAL
	sound.init();

	//Set up physics
	PhysicsInterface physInterface;
	dynamicsWorld = physInterface.init();

	//btCollisionShape *armShape = new btSphereShape(.2);
	//KinematicMotionState *rArmMotionState = new KinematicMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0, 2, 0)));
	//btRigidBody::btRigidBodyConstructionInfo rArmRigidBodyCI(0, rArmMotionState, armShape, btVector3(0,0,0));
	//rightArm = new btRigidBody(rArmRigidBodyCI);
	//Make the arm kinematic
	//rightArm->setCollisionFlags(rightArm->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//rightArm->setActivationState(DISABLE_DEACTIVATION);
	//dynamicsWorld->addRigidBody(rightArm);
	player = physInterface.initPlayer(btVector3(playerPos[0], playerPos[1], playerPos[2]));

	//Set up OGRE
	GraphicsInterface graphInterface;
	root = graphInterface.init();
	Ogre::RenderWindow *ogreWindow = graphInterface.GetWindow("Project 2");
	graphInterface.SetUpCamera();
	//graphInterface.GetManager()->setSkyDome(true, "SkyDome", 10.f, 1.f);
	createSphere("mySphereMesh", 40000, 64, 64);
	Entity* sphereEntity = graphInterface.GetManager()->createEntity("mySphereEntity", "mySphereMesh");
	SceneNode* sphereNode = graphInterface.GetManager()->getRootSceneNode()->createChildSceneNode();
	sphereEntity->setMaterialName("SkyDome");
	sphereEntity->setRenderingDistance(100000);
	sphereEntity->setCastShadows(false);
	sphereNode->attachObject(sphereEntity);

	//Load Rescue ship
	Ogre::Entity *novaMesh = graphInterface.GetManager()->createEntity("novaMesh", "Argus.mesh");
	Ogre::SceneNode *nova = graphInterface.GetRootSceneNode()->createChildSceneNode("nova");
	novaMesh->setCastShadows(false);
	nova->setPosition(-2483.82, 570.494, -619.727);
	nova->setOrientation(0.923881, 0, 0.382684, 0);
	//nova->setPosition(-2310.82, 5.49368, -619.727);
	nova->setScale(6, 6, 6);
	nova->attachObject(novaMesh);

	//Load player ship components
	Ogre::Entity *saxonBody = graphInterface.GetManager()->createEntity("saxonBody", "Saxon1.mesh");
	Ogre::SceneNode *shipBody = graphInterface.GetRootSceneNode()->createChildSceneNode("Body");
	saxonBody->setCastShadows(false);
	shipBody->scale(6, 6, 6);
	shipBody->attachObject(saxonBody);
	shipBody->setPosition(-85, 100, -100);

	Ogre::Entity *saxonForward = graphInterface.GetManager()->createEntity("saxonForwardWeapons", "Saxon2_1.mesh");
	saxonForward->setCastShadows(false);
	Ogre::SceneNode *shipForward = shipBody->createChildSceneNode("ForwardWeapons");
	shipForward->attachObject(saxonForward);

	Ogre::Entity *saxonMid = graphInterface.GetManager()->createEntity("saxonMid", "Saxon2_2.mesh");
	saxonMid->setCastShadows(false);
	Ogre::SceneNode *shipMid = shipBody->createChildSceneNode("MidWeapons");
	shipMid->attachObject(saxonMid);

	Ogre::Entity *saxonAft = graphInterface.GetManager()->createEntity("saxonAft", "Saxon2_3.mesh");
	saxonAft->setCastShadows(false);
	Ogre::SceneNode *shipAft = shipBody->createChildSceneNode("AftWeapons");
	shipAft->attachObject(saxonAft);

	Ogre::Entity *saxonLounge = graphInterface.GetManager()->createEntity("saxonLounge", "Saxon2_4.mesh");
	saxonLounge->setCastShadows(false);
	Ogre::SceneNode *shipLounge = shipBody->createChildSceneNode("ShipLounge");
	shipLounge->attachObject(saxonLounge);

	Ogre::Entity *saxonBay = graphInterface.GetManager()->createEntity("saxonBay", "Saxon3.mesh");
	saxonBay->setCastShadows(false);
	Ogre::SceneNode *shipBay = shipBody->createChildSceneNode("Bay");
	shipBay->attachObject(saxonBay);

	Ogre::Entity *saxonShips = graphInterface.GetManager()->createEntity("saxonShips", "Saxon4.mesh");
	saxonShips->setCastShadows(false);
	Ogre::SceneNode *shipShips = shipBay->createChildSceneNode("Ships");
	shipShips->attachObject(saxonShips);

	//Place cargo in the hold
	Ogre::Entity *cargoMesh = graphInterface.GetManager()->createEntity("cargoMesh", "cargo.mesh");
	std::list<btRigidBody*> *cargoBoxes = placeCargo(dynamicsWorld);
	std::list<btRigidBody*>::iterator cargoIT = cargoBoxes->begin();
	for(int i = 0; i < cargoBoxes->size(); i++, cargoIT++)
	{
		char name[50];
		sprintf(name, "cargoBox%d", i);
		Ogre::Entity *newBox = cargoMesh->clone(name);
		newBox->setMaterialName("Cargo");
		sprintf(name, "cargoBoxScene%d", i);
		Ogre::SceneNode *newBoxSN = shipBay->createChildSceneNode(name);
		newBoxSN->attachObject(newBox);
		newBoxSN->setScale(3, 3, 3);

		PhysicsObjects.push_back(*cargoIT);
		OgreObjects.push_back(newBoxSN);
	}

	delete cargoBoxes;

	#pragma region Set up OpenAL
	for(int i = 0; i < SILENCEINT; ++i)
	{
		sound.createSound(SoundString[i], SoundString[i]);
	}
	#pragma endregion

	//Get nodes and set initial position
	Ogre::SceneNode *playerNode = (SceneNode*)graphInterface.GetRootSceneNode()->getChild("Player");
	Ogre::SceneNode *mainCamera = (SceneNode*)playerNode->getChild("main_camera");
	//mainCamera->setPosition(431.265, -16.1224, -1106.07);
	mainCamera->yaw(Degree(180));

	//Particle Effect
	Ogre::ParticleSystem *system = graphInterface.GetManager()->createParticleSystem("ParticleSys", "ForceField");
	Ogre::SceneNode *particles = graphInterface.GetRootSceneNode()->createChildSceneNode("Particles");
	particles->setPosition(shipBody->getPosition() + Vector3(660, -100, -125 * 6));
	Ogre::ParticleAffector *affector = system->getAffector(0);
	Vector3 tmpParticlePos = particles->getPosition() + Vector3(25, 0, 0);
	char tmpString[255];
	sprintf(tmpString, "%f %f %f", tmpParticlePos.x, tmpParticlePos.y, tmpParticlePos.z);
	affector->setParameter("plane_point", tmpString);
	particles->rotate(Vector3::UNIT_Y, Degree(180));
	particles->attachObject(system);
	system->fastForward(10);

	//Rear forcefield
	Ogre::ParticleSystem *system2 = graphInterface.GetManager()->createParticleSystem("ParticleSys2", "ForceField");
	Ogre::SceneNode *particles2 = graphInterface.GetRootSceneNode()->createChildSceneNode("Particles2");
	particles2->setPosition(shipBody->getPosition() + Vector3(660, -100, -205 * 6));
	system2->removeAllAffectors();
	particles2->rotate(Vector3::UNIT_Y, Degree(180));
	particles2->attachObject(system2);
	system2->fastForward(10);

	//Warp exit particle system
	Ogre::ParticleSystem *warpExit = graphInterface.GetManager()->createParticleSystem("WarpExit", "Warp");
	Ogre::SceneNode *warpSN = graphInterface.GetRootSceneNode()->createChildSceneNode("WarpExitSN");
	warpSN->setPosition(-1289.07, 573.749, -2682.98);
	warpSN->attachObject(warpExit);
	warpSN->setOrientation(nova->getOrientation());
	warpExit->setVisible(false);
	warpExit->getEmitter(0)->setParameter("velocity", ".1");
	warpExit->getEmitter(0)->setParameter("velocity_min", ".1");
	warpExit->getEmitter(0)->setParameter("velocity_max", ".1");
	warpExit->getEmitter(0)->setParameter("emission_rate", "5000");

	loadObj("Saxon/Saxon1.obj", dynamicsWorld, btVector3(-85, 100, -100), 600.f);
	//loadObj("Saxon/Saxon2.obj", dynamicsWorld, btVector3(-85, 100, -100), 1200.f);
	loadObj("Saxon/Saxon3.obj", dynamicsWorld, btVector3(-85, 100, -100), 60.f);
	loadObj("Saxon/Saxon4.obj", dynamicsWorld, btVector3(-85, 100, -100), 60.f);

	//Add bay lighting to the Saxon
	Ogre::Light *bayLight = graphInterface.GetManager()->createLight("bayLight");
	bayLight->setPosition(431.443, 4.69853, -1072.22);
	graphInterface.GetRootSceneNode()->attachObject(bayLight);//TODO: Make this relative to the ship bay

	//Add a light to the player
	Ogre::Light *l = graphInterface.GetManager()->createLight("light1");
	l->setType(Light::LT_SPOTLIGHT);
	l->setVisible(false);
	l->setDirection(Vector3(0, 0, -1));
	mainCamera->attachObject(l);

	//Add some boxes
	btCollisionShape *blockShape = new btBoxShape(btVector3(1,.5,1));
	btVector3 fallInertia(0,0,0);

	//Set debugging
	dbgdraw = new BtOgre::DebugDrawer(graphInterface.GetRootSceneNode(), dynamicsWorld);
	dynamicsWorld->setDebugDrawer(dbgdraw);

	//Allow for keyboard control
	size_t hWnd = 0;
	ogreWindow->getCustomAttribute("WINDOW", &hWnd);
	OIS::InputManager *m_InputManager = OIS::InputManager::createInputSystem(hWnd);
	OIS::Keyboard *m_Keyboard = static_cast<OIS::Keyboard*>(m_InputManager->createInputObject(OIS::OISKeyboard, false));
	char *keyStates = new char[512];
	
	//load the font
	Ogre::FontPtr mFont = Ogre::FontManager::getSingleton().create("bluehigh", "General");
	mFont->setType(Ogre::FT_TRUETYPE);
	mFont->setSource("Fonts/bluehigh.ttf");
	mFont->setTrueTypeSize(36);
	mFont->setTrueTypeResolution(96);
	mFont->addCodePointRange(Ogre::Font::CodePointRange(33, 255));
	
	//create material
	Ogre::MaterialPtr mMat = Ogre::MaterialManager::getSingleton().create("mMaterial", "General", true);
	Ogre::Technique* mTech = mMat->createTechnique();
	Ogre::Pass* mPass = mTech->createPass();
	Ogre::TextureUnitState* mTexUnitState = mPass->createTextureUnitState();
	mPass = mMat->getTechnique(0)->getPass(0);
	mPass->setDiffuse(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 0.9f));

	//load texture
	Ogre::TexturePtr mTex = Ogre::TextureManager::getSingleton().load("hudbackground2.png","General");
	mPass->createTextureUnitState()->setTextureName(mTex->getName());
	mPass->setCullingMode(Ogre::CULL_NONE);                
	mPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	mMat->setLightingEnabled(false);   
	mMat->setDepthCheckEnabled(false); 

	OverlayManager& overlayManager = OverlayManager::getSingleton();
	//creates the left panel
	OverlayContainer* panelLeft = static_cast<OverlayContainer*>(overlayManager.createOverlayElement("Panel", "LeftPanelName"));
	panelLeft->setMetricsMode(Ogre::GMM_PIXELS);
	panelLeft->setPosition(0, 550);
	panelLeft->setDimensions(240, 50);
	panelLeft->setMaterialName("mMaterial"); 

	//creates the right panel
	OverlayContainer* panelRight = static_cast<OverlayContainer*>(overlayManager.createOverlayElement("Panel", "RightPanelName"));
	panelRight->setMetricsMode(Ogre::GMM_PIXELS);
	panelRight->setPosition(560, 550);
	panelRight->setDimensions(240, 50);
	panelRight->setMaterialName("mMaterial"); 
 
	//creates the left text area
	TextAreaOverlayElement* textAreaLeft = static_cast<TextAreaOverlayElement*>(
		overlayManager.createOverlayElement("TextArea", "LeftTextAreaName"));
	textAreaLeft->setMetricsMode(Ogre::GMM_PIXELS);
	textAreaLeft->setPosition(10, 0);
	textAreaLeft->setDimensions(500, 1000);
	textAreaLeft->setCharHeight(24);
	textAreaLeft->setFontName("bluehigh");
	textAreaLeft->setColourBottom(ColourValue(0.5, 0.5, 0.9));
	textAreaLeft->setColourTop(ColourValue(0.5, 0.5, 0.7));

	//creates the right text area
	TextAreaOverlayElement* textAreaRight = static_cast<TextAreaOverlayElement*>(
		overlayManager.createOverlayElement("TextArea", "RightTextAreaName"));
	textAreaRight->setMetricsMode(Ogre::GMM_PIXELS);
	textAreaRight->setPosition(10, 0);
	textAreaRight->setDimensions(1200, 1000);
	textAreaRight->setCharHeight(16);
	textAreaRight->setFontName("bluehigh");
	textAreaRight->setColourBottom(ColourValue(0.5, 0.5, 0.9));
	textAreaRight->setColourTop(ColourValue(0.5, 0.5, 0.7));
 
	// Create an overlay, and add the panels
	Overlay* leftOverlay = overlayManager.create("LeftOverlayName");
	leftOverlay->add2D(panelLeft);

	Overlay* rightOverlay = overlayManager.create("RightOverlayName");
	rightOverlay->add2D(panelRight);
 
	// Add the text area to the panel
	panelLeft->addChild(textAreaLeft);
	panelRight->addChild(textAreaRight);
 
	// Show the overlay
	leftOverlay->show();
	rightOverlay->show();

	//Set up objectives
	string objectives[] = {"Restore Emergency Power", "Disable Plasma Forcefields", "Send S.O.S. from Comm", "You Win!"};
	Vector3 objLocations[] = {Vector3(500.076, 2.85797, -1264.22), Vector3(510.231, 7.43374, -904.908), Vector3(398.088, 455.44, -1012.31), Vector3(398.088, 455.44, -1012.31)};
	double distanceToObjective = 9999;
	int currentObjective = 0;
	
#pragma region Main Loop
	int count = 1;
	GameTimer timer;
	while(1)
	{	
		//Updates
		double elapsed = timer.getElapsedTimeSec();
		graphInterface.RenderFrame(elapsed);//Draw frames
		Ogre::WindowEventUtilities::messagePump();
		dynamicsWorld->stepSimulation(elapsed);
		//dbgdraw->step();
		btVector3 playerForce = btVector3(0,0,0);

		double speedPerFrame = MOVEMENT_SPEED * elapsed;
		
		distanceToObjective = playerNode->getPosition().distance(objLocations[currentObjective]);

		textAreaLeft->setCaption("Suit Integrity:\tGood"
						"\nOxygen Level:\tHigh");

		char numBuffer[4];
		sprintf(numBuffer, "%.2f", distanceToObjective);
		textAreaRight->setCaption("Objective:\t" + objectives[currentObjective] + 
					"\nDistance:\t" + numBuffer);

		//Keyboard check
		m_Keyboard->capture();
		if(m_Keyboard->isKeyDown(OIS::KC_W))
			playerForce.setZ(-speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_S))
			playerForce.setZ(speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_A))
			playerForce.setX(-speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_D))
			playerForce.setX(speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_N) && !keyStates[OIS::KC_N])
			bayLight->setDiffuseColour(.1, .1, .1);
		if(m_Keyboard->isKeyDown(OIS::KC_M) && !keyStates[OIS::KC_M])
			bayLight->setDiffuseColour(1, 0, 0);
		if(m_Keyboard->isKeyDown(OIS::KC_DOWN))
			mainCamera->pitch(Ogre::Degree(elapsed * 50));
		if(m_Keyboard->isKeyDown(OIS::KC_UP))
			mainCamera->pitch(Ogre::Degree(elapsed * -50));
		if(m_Keyboard->isKeyDown(OIS::KC_SPACE))
			playerForce.setY(speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_LCONTROL))
			playerForce.setY(-speedPerFrame);
		if(m_Keyboard->isKeyDown(OIS::KC_RCONTROL))
			cout << playerNode->getPosition() << endl;
		if(m_Keyboard->isKeyDown(OIS::KC_B))
		{
			initialFailure();
		}

		//--------------------------------------------/
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPADENTER))
		{
			cout << nova->getPosition() << endl;
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD0))
		{
			cout << nova->getOrientation() << endl;
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD8))
		{
			nova->translate(0, 1, 0);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD2))
		{
			nova->translate(0, -1, 0);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD4))
		{
			nova->translate(-1, 0, 0);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD6))
		{
			nova->translate(1, 0, 0);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD9))
		{
			nova->yaw(Degree(3));
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD7))
		{
			nova->yaw(Degree(-3));
		}
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD1))
			nova->translate(nova->getOrientation() * Vector3::UNIT_Z * 1);
		if(m_Keyboard->isKeyDown(OIS::KC_NUMPAD3))
			nova->translate(nova->getOrientation() * Vector3::UNIT_Z * 3);

		if(m_Keyboard->isKeyDown(OIS::KC_INSERT))
		{
			warpExit->setEmitting(true);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_PGUP))
		{
			warpExit->setEmitting(false);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_HOME) && !keyStates[OIS::KC_HOME])
			nova->flipVisibility();
		if(m_Keyboard->isKeyDown(OIS::KC_DELETE))
		{
			warpExit->getEmitter(0)->setParameter("velocity", ".1");
			warpExit->getEmitter(0)->setParameter("velocity_min", ".1");
			warpExit->getEmitter(0)->setParameter("velocity_max", ".1");
			warpExit->getEmitter(0)->setParameter("emission_rate", "5000");
		}
		if(m_Keyboard->isKeyDown(OIS::KC_END))
		{
			warpExit->getEmitter(0)->setParameter("velocity", "400");
			warpExit->getEmitter(0)->setParameter("velocity_min", "400");
			warpExit->getEmitter(0)->setParameter("velocity_max", "600");
			warpExit->getEmitter(0)->setParameter("emission_rate", "2000");
		}
		//--------------------------------------------/

		//Trigger objective event
		if(m_Keyboard->isKeyDown(OIS::KC_E) && distanceToObjective < 20)
		{
			if(currentObjective == 3){}
			else
			{
				if(currentObjective == 0)
				{
					bayLight->setDiffuseColour(1, 0, 0);
				}
				else if(currentObjective == 1)
				{
					system->setEmitting(false);
					system2->setEmitting(false);
				}
				else if(currentObjective == 2)
				{
					warpExit->setVisible(true);

				}
				currentObjective++;
			}


		}

		//Flashlight On/Off
		if(m_Keyboard->isKeyDown(OIS::KC_F) && !keyStates[OIS::KC_F])
			l->setVisible(!l->getVisible());

		//Turn off plasma fields
		if(m_Keyboard->isKeyDown(OIS::KC_G) && !keyStates[OIS::KC_G])
		{
			system->setEmitting(!system->getEmitting());
			system2->setEmitting(!system2->getEmitting());
		}

		//Slow down the player
		if(m_Keyboard->isKeyDown(OIS::KC_LSHIFT))
		{
			player->setLinearVelocity(player->getLinearVelocity() * .9f);
		}

		//Align force to camera
		Vector3 OgrePlayerForce(playerForce.x(), playerForce.y(), playerForce.z());
		OgrePlayerForce = mainCamera->getChild("cameraPitch")->_getDerivedOrientation() * OgrePlayerForce;
		playerForce = btVector3(OgrePlayerForce.x, OgrePlayerForce.y, OgrePlayerForce.z);

		player->applyCentralForce(playerForce);
		//player->setLinearVelocity(playerForce);
		//graphInterface.MoveCameraForward(playerForce, 1);

		if(m_Keyboard->isKeyDown(OIS::KC_Q))//rotate left
		{
			Ogre::Quaternion quat(Ogre::Degree(elapsed * 50), Ogre::Vector3::UNIT_Y);
			mainCamera->rotate(quat);
		}
		if(m_Keyboard->isKeyDown(OIS::KC_E))//rotate right
		{
			Ogre::Quaternion quat(Ogre::Degree(elapsed * -50), Ogre::Vector3::UNIT_Y);
			mainCamera->rotate(quat);
		}
#pragma region Fire block code
		if(fireTimer > FIRE_WAIT_TIME)
		{
			if(m_Keyboard->isKeyDown(OIS::KC_RETURN))//Fire projectile
			{
				//Find the orientation of the camera, set it up as a direction
				Ogre::Quaternion q = mainCamera->getOrientation(); //Get the quaternion from the camera

				//Spawn a projectile object facing that direction (+1 unit from where you are facing) away from the right arm
				Ogre::Vector3 v3One(0, 0, -1);
				v3One = mainCamera->_getDerivedOrientation() * mainCamera->_getDerivedOrientation() * v3One + mainCamera->_getDerivedPosition();
				btDefaultMotionState *projectileMotionState = new btDefaultMotionState(btTransform(btQuaternion(q.w, q.x, q.y, q.z), btVector3(v3One.x, v3One.y, v3One.z)));
				btScalar boxMass = 10;
				blockShape->calculateLocalInertia(boxMass, fallInertia);
				btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(boxMass, projectileMotionState, blockShape, fallInertia);
				btRigidBody *newBox = new btRigidBody(boxRigidBodyCI);
				newBox->setRestitution(.9f);
				
				Ogre::Vector3 projectileTemp(q * mainCamera->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z * 200);
				btVector3 projectileVelocity(projectileTemp.x, projectileTemp.y, projectileTemp.z);
				newBox->setLinearVelocity(projectileVelocity);

				dynamicsWorld->addRigidBody(newBox);
				PhysicsObjects.push_back(newBox);

				Ogre::SceneNode *root_sn = graphInterface.GetRootSceneNode();
				Ogre::SceneManager *manager = graphInterface.GetManager();
				char name[20];
				sprintf(name, "projectile%d", projectileNum);
				++projectileNum;
				Ogre::Entity *projectileO = manager->createEntity(name, "cube.mesh");
				projectile_sn = root_sn->createChildSceneNode(name);
				projectile_sn->scale(.0025, .0025, .0025);
				projectile_sn->attachObject(projectileO);
				projectileO->setCastShadows(true);
				projectileO->setMaterialName("M_NoLighting");

				OgreObjects.push_back(projectile_sn);

				fireTimer -= FIRE_WAIT_TIME;
			}
		}
		else
		{
			fireTimer += elapsed;
		}
#pragma endregion

		//Update Ogre based on physics
		std::list<btRigidBody*>::iterator Pit;
		Pit = PhysicsObjects.begin();
		std::list<SceneNode*>::iterator Oit;
		Oit = OgreObjects.begin();
		for(int i = 0; i < PhysicsObjects.size(); i++, Pit++, Oit++)
		{
			btTransform tr;
			(*Pit)->getMotionState()->getWorldTransform(tr);

			(*Oit)->_setDerivedPosition(Vector3(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ()));
			(*Oit)->setOrientation(tr.getRotation().getW(), tr.getRotation().getX(), tr.getRotation().getY(), tr.getRotation().getZ());
		}

		btVector3 playerPhysPos = player->getWorldTransform().getOrigin();
		btVector3 playerVel = player->getLinearVelocity();
		playerNode->setPosition(playerPhysPos.x(), playerPhysPos.y(), playerPhysPos.z());
		//cout << playerVel.x() << " " << playerVel.y() << " " << playerVel.z() << endl;

		//Sleep(elapsed);
		m_Keyboard->copyKeyStates(keyStates);

		if (dynamicsWorld)
			dynamicsWorld->performDiscreteCollisionDetection();
		CollisionFeedback();

		if(soundVar < RANDOMSTARTINT)
		{
			mainPlaySound(elapsed);
		}
		else if(soundVar >= RANDOMSTARTINT && !soundBool)
		{
			if(soundVar != SILENCEINT) soundVar = (rand() % (SILENCEINT - RANDOMSTARTINT)) + RANDOMSTARTINT;
			double sPosition[] = {mainCamera->_getDerivedPosition().x, mainCamera->_getDerivedPosition().y, mainCamera->_getDerivedPosition().z};
			playSound(elapsed, soundVar, sPosition, false);
		}
		else
		{
			double sPosition[] = {mainCamera->_getDerivedPosition().x, mainCamera->_getDerivedPosition().y, mainCamera->_getDerivedPosition().z};
			playSound(elapsed, 0, sPosition, true);
		}
		//printf("Mainloop soundVar: %d.\n", soundVar);

		if(count % 10 == 0)
		{
			cout << "FPS: " << 1.0 / elapsed << endl;
		}
		count++;





	}
#pragma endregion
}