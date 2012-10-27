#include "PhysicsInterface.h"

btDiscreteDynamicsWorld* PhysicsInterface::init()
{
	btBroadphaseInterface * broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher * dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver * solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.8, 0));

	return dynamicsWorld;
}

btRigidBody* PhysicsInterface::initPlayer(btVector3 &startPos)
{
	btCollisionShape* playerShp = new btCapsuleShape(1, 5);
	btDefaultMotionState *playerST = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),startPos));
	btScalar mass = 100;
	btVector3 playerInertia(0,0,0);
	playerShp->calculateLocalInertia(mass, playerInertia);
	btRigidBody::btRigidBodyConstructionInfo playerCI(mass, playerST, playerShp, playerInertia);
	btRigidBody *player = new btRigidBody(playerCI);
	dynamicsWorld->addRigidBody(player);
	player->setAngularFactor(0.0f);//disable rotation
	player->setActivationState(DISABLE_DEACTIVATION);

	return player;
}