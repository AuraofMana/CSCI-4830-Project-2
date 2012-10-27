#include "PhysicsStructs.h"

PhysicsLoader::PhysicsLoader(TiXmlDocument *doc)
{
	TiXmlHandle hDoc(doc);
	TiXmlElement *pElem;
	TiXmlHandle hRoot(0);

	pElem = hDoc.FirstChildElement().Element();
	string testName = pElem->Value();

	hRoot = TiXmlHandle(pElem);

	//Get the sub meshes
	pElem = hRoot.FirstChildElement().FirstChildElement().Element();
	for(; pElem; pElem = pElem->NextSiblingElement())
	{
		subMesh newMesh;
		newMesh.material = pElem->Attribute("material");
		newMesh.usesharedvertices = false; //TODO: Check for this if it's ever useful

		//Get the faces of the mesh
		TiXmlElement *faces = pElem->FirstChildElement("faces")->FirstChildElement();
		for(; faces; faces = faces->NextSiblingElement())
		{
			face newFace;
			newFace.v1 = atoi(faces->Attribute("v1"));
			newFace.v2 = atoi(faces->Attribute("v2"));
			newFace.v3 = atoi(faces->Attribute("v3"));

			newMesh.faces.push_back(newFace);
		}

				//Get the vertices of the mesh
		TiXmlElement *vertices = pElem->FirstChildElement("geometry")->FirstChildElement()->FirstChildElement();
		for(; vertices; vertices = vertices->NextSiblingElement())
		{
			vertex newVertex;
			newVertex.pos.x = strtod(vertices->FirstChildElement("position")->Attribute("x"), NULL);
			newVertex.pos.y = strtod(vertices->FirstChildElement("position")->Attribute("y"), NULL);
			newVertex.pos.z = strtod(vertices->FirstChildElement("position")->Attribute("z"), NULL);

			newVertex.norm.x = strtod(vertices->FirstChildElement("normal")->Attribute("x"), NULL);
			newVertex.norm.y = strtod(vertices->FirstChildElement("normal")->Attribute("y"), NULL);
			newVertex.norm.z = strtod(vertices->FirstChildElement("normal")->Attribute("z"), NULL);

			newMesh.vertices.push_back(newVertex);

		}

		this->submeshes.push_back(newMesh);
		
	}
}

void PhysicsLoader::AddMeshes(btDynamicsWorld *dynamicsWorld, btVector3 &position)
{
	for(int i = 0; i < this->submeshes.size(); i++)
	{
		subMesh *mesh = &submeshes[i];

		vector<btTriangleMesh*> triMeshes;
		btTriangleMesh *triMesh;// = new btTriangleMesh();
		//triMeshes.push_back(triMesh);

		//std::cout << mesh->faces.size() << std::endl;
		//Find the vertices of each triangle
		int count = 0;
		for(int j = 0; j < mesh->faces.size(); j++, count++)
		{
			if(mesh->faces[j].v1 == -1)
			{
				count--;
				continue;
			}
			if(count % 100 == 0)
			{
				triMesh = new btTriangleMesh();
				triMeshes.push_back(triMesh);
			}

			int idx1 = mesh->faces[j].v1;
			int idx2 = mesh->faces[j].v2;
			int idx3 = mesh->faces[j].v3;

			btVector3 vertex1(mesh->vertices[idx1].pos.x, mesh->vertices[idx1].pos.y, mesh->vertices[idx1].pos.z);
			btVector3 vertex2(mesh->vertices[idx2].pos.x, mesh->vertices[idx2].pos.y, mesh->vertices[idx2].pos.z);
			btVector3 vertex3(mesh->vertices[idx3].pos.x, mesh->vertices[idx3].pos.y, mesh->vertices[idx3].pos.z);

			triMesh->addTriangle(vertex1, vertex2, vertex3);
		}

		for(int j = 0; j < triMeshes.size(); j++)
		{
			triMesh = triMeshes[j];
			btConvexShape *tmpshape = new btConvexTriangleMeshShape(triMesh);

			btShapeHull* hull = new btShapeHull(tmpshape);
			btScalar margin = tmpshape->getMargin();
			hull->buildHull(margin);
			btConvexHullShape* simplifiedConvexShape = new btConvexHullShape((const btScalar *)hull->getVertexPointer(),hull->numVertices());

			btDefaultMotionState *motionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), position));
			btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, motionState, tmpshape, btVector3(0,0,0));
			btRigidBody *rigidBody = new btRigidBody(rigidBodyCI);
			dynamicsWorld->addRigidBody(rigidBody);
		}
	}

}

bool PhysicsLoader::CleanMeshes(btScalar distanceAllowed)
{
	//loop through submeshes
	for(int i = 0; i < this->submeshes.size(); i++)
	{
		int size = this->submeshes[i].vertices.size();
		btVector3 *vertices = new btVector3[size];
		for(int j = 0; j < size; j++)
		{
			vertices[j] = btVector3(submeshes[i].vertices[j].pos.x, submeshes[i].vertices[j].pos.y, submeshes[i].vertices[j].pos.z);
		}

		int *map = new int[size];
		//Dreaded n^2 loop
		for(int j = 0; j < size; j++)
		{
			if(vertices[j].length() == 0) continue;// Already removed
			for(int k = 0; k < size; k++)
			{
				if(j == k) continue;//It would remove itself
				if(vertices[k].length() == 0) continue;//Already removed

				if(vertices[j].distance(vertices[k]) < distanceAllowed)
				{
					vertices[k] = btVector3(0, 0, 0);
					map[k] = j;
				}
			}
		}
		//Update faces
		for(int j = 0; j < this->submeshes[i].faces.size(); j++)
		{
			face *checkFace = &(this->submeshes[i].faces[j]);

			int errorCount = 0;
			if(vertices[checkFace->v1].length() == 0) errorCount++;
			if(vertices[checkFace->v2].length() == 0) errorCount++;
			if(vertices[checkFace->v3].length() == 0) errorCount++;

			if(errorCount >= 2)
				checkFace->v1 = -1;
			else
			{
				if(vertices[checkFace->v1].length() == 0) checkFace->v1 = map[checkFace->v1];
				if(vertices[checkFace->v2].length() == 0) checkFace->v2 = map[checkFace->v2];
				if(vertices[checkFace->v3].length() == 0) checkFace->v3 = map[checkFace->v3];
			}
		}
	}

	return false;
}