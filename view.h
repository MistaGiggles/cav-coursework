#ifndef _rt_H
#define _rt_H

#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <cstring>
#include <GL/glut.h>
#include "matrix4f.h"


using namespace std;

//
// Sample code for physics simulation
//


// Implements cloth simulation


class Vector3f;
class Triangle;
class TriangleMesh;
class Edge;

class Skeleton;
class Bone;
class Joint;
class Weight;

class Joint {
	int id;
	float x, y, z;
	Vector3f position;


public:
		int connected;

	Joint(int _id, float _x, float _y, float _z, int _con)
	{
		id = _id;
		x = _x;
		y = _y;
		z = _z;
		position[0] = x;
		position[1] = y;
		position[2] = z;
		connected = _con;
	};

	Vector3f& getPosition() {
		return position;
	}


};

class Weight {
	std::vector<float> weightings;
	int last;
public:
	Weight(int size) : weightings(size) {
	};
	int getLast()
	{
		return last;
	}
	void addWeight(float v, int i) {
		weightings[i] = v;
		if(v > 0 && i > last)
		{
			last = i;
		}
	};

	float operator [](int i) {
		return weightings[i];
	}
};

class Bone {

public:

	Vector3f Coords;
	Matrix4f Rotation;
	Matrix4f GlobalPosition;
	Matrix4f LocalTranslation;
	Matrix4f NewLocalTranslation;


	

	Bone* Parent;
	bool isRoot;
	int id;

	Bone(float x, float y, float z, Bone* _Parent) 
	{
		
		Coords[0] = x; Coords[1] = y; Coords[2] = z;
		Parent = _Parent;
		//GlobalPosition.setTranslation(Coords);
		//LocalTranslation.setTranslation(Coords - Parent->Coords);
		//Rotation.setIdentity();
		isRoot = false;
	};

	Bone(float x, float y, float z)
	{
		
		Coords[0] = x; Coords[1] = y; Coords[2] = z;
		//GlobalPosition.setTranslation(Coords);
		//LocalTranslation = GlobalPosition;
		//Rotation.setIdentity();
		isRoot = true;
	}

	void setID(int _id)
	{
		if(_id > 0)
		{
			isRoot = false;
		}
		id = _id;
	};

	int getID()
	{
		return id;
	};
	
	void setRotation(double rotx, double roty, double rotz)
	{
		Rotation = rotX(rotx) * rotY(roty) * rotZ(rotz);
	};

	void InitGlobal()
	{
		Rotation.setIdentity();
		GlobalPosition.setIdentity();
		GlobalPosition.setTranslation(Coords);
		LocalTranslation.setIdentity();
		NewLocalTranslation.setIdentity();
		//NewLocalTranslation.setTranslation(Vector3f(0.1f,0.1f,0.1f));

		if(!isRoot)
		{
			LocalTranslation.setTranslation(Coords - Parent->Coords);
			
		} else {
			LocalTranslation.setTranslation(Coords);
		}


		//GlobalPosition.print();
		//std::cout<<"^GLOB   |   v LOCAL"<<std::endl;
		//LocalTranslation.print();
		
	}

	void Chain(Matrix4f& m)
	{
		if(isRoot)
		{
			m = m * Rotation * NewLocalTranslation;
		} else {
			Parent->Chain(m);
			m = m * Rotation * NewLocalTranslation;
		}
		return;
	};

	void restChain(Matrix4f& m)
	{
		if(isRoot)
		{
			m = m * LocalTranslation;
		} else {
			Parent->restChain(m);
			m = m * LocalTranslation;
		}
		return;
	};

	
};


class Skeleton {

	std::vector<Bone*> bones;
	std::vector<Weight> weights;
	float curRot;
	int parentBone;
public:
	Skeleton()
	{
		curRot = 0;
	}

	void draw()
	{
			curRot += 0.00001f;
			//bones[parentBone]->setRotation(0,0,curRot);
			bones[20]->setRotation(0,0,curRot);
			for(int i = 0; i < bones.size(); i++)
			{
				if(i != parentBone)
				{
					glBegin(GL_LINES);
						glColor3f(1.0,1.0,1.0);
						Matrix4f p1;
						Matrix4f p2;
						Matrix4f rp1Inv;
						Matrix4f rp2Inv;

						
						p1.setIdentity();
						//p1.setTranslation(Vector3f(0,0,0));
						p2.setIdentity();
						//p2.setTranslation(Vector3f(0,0,0));
						//p1.print();

						bones[i]->restChain(p1);
						bones[i]->Parent->restChain(p2);
						p1.inv(p1, rp1Inv);
						p2.inv(p2, rp2Inv);

						p1.setIdentity();
						p2.setIdentity();

						bones[i]->Chain(p1);
						bones[i]->Parent->Chain(p2);

						Vector3f a(0,0,0);
						Vector3f b(0,0,0);
						a = p1 * rp1Inv * a;
						b = p2 * rp2Inv * b;
						glVertex3f(a[0], a[1], a[2]);
						glVertex3f(b[0], b[1], b[2]);
						//glVertex3f(p1(0,3), p1(1,3), p1(2,3));
						//std::cout<<"Original x,y,z: "<<a[0]<<", "<<a[1]<<", "<<a[2]<<", "<<std::endl;
						//std::cout<<"Chained  x,y,z: "<<p1(0,3)<<", "<<p1(1,3)<<", "<<p1(2,3)<<", "<<std::endl;
						//p1.print();
						//Vector3f b = bones[i]->Parent->Coords;
						//if(bones[i]->Parent->isRoot){std::cout<<i<<" is a child of root "<<bones[i]->Parent->getID()<<std::endl;}
						//std::cout<<"Bone: "<<i<<" coords: "<<b[0]<<", "<<b[1]<<", "<<b[2]<<std::endl;
						//glVertex3f(p2(0,3), p2(1,3), p2(2,3));
						
						
						
					glEnd();
				}

			}
		
	}

	void Rig(Vector3f& v, int vi)
	{
		Weight w = weights[vi];
		
		Vector3f pos;
		
		for(int i = 0; i < bones.size(); i++)
		{
			if(w[i] >0)
			{
				//std::vector<int> path = getPath(w.getlast());
				//for(int j = 0; j < path.size();p j++)
				//{
				//	int bone
				//}
				//pos +=  bones[i].Transformation(v) * w[i];
			}
		}
		
		//v[0] = pos[0]*27.0f;
		//v[1] = pos[1]*27.0f;
		//v[2] = pos[2]*27.0f;

		//std::cout<<"v1: "<<v[0]<<" : "<<pos[0]<<" : "<<pos[0]/v[0]<<std::endl;
		//std::cout<<"v2: "<<v[1]<<" : "<<pos[1]<<" : "<<pos[0]/v[0]<<std::endl;
		//std::cout<<"v3: "<<v[2]<<" : "<<pos[2]<<" : "<<pos[0]/v[0]<<std::endl;
	}

	

	void load(char* skeleton_file, char* weights_file) 
	{
		ifstream skele_file(skeleton_file);
		if(skele_file==NULL) cerr << "failed reading skeleton data file " << skeleton_file;
		char buf[1024];
		// id, x, y, z, connected
		int id, connected;
		float x, y, z;
		id = 0;
		while(!skele_file.eof()) {
			skele_file.getline(buf, sizeof(buf));
			id = -99;
			sscanf(buf, "%i %f %f %f %i", &id, &x, &y, &z, &connected);
			if(id == bones.size()) {
				
				if(connected == -1)
				{
					std::cout<<"Created Root bone: "<<id<<std::endl;
					Bone* b = new Bone(x,y,z);
					b->setID(id);
					bones.push_back(b);
					parentBone = id;
				} else {
					std::cout<<"Created bone: "<<connected<<"<-"<<id<<" coords: "<<x<<", "<<y<<", "<<z<<std::endl;
					if(bones[connected]->getID()==connected)
					{

					} else
					{
						std::cout<<"Tried to connect bone to incorrect paretn"<<std::endl;
					}
					Bone* b = new Bone(x,y,z, bones[connected]);
					b->setID(id);
					bones.push_back(b);
				}
				

				//Joint j(id, x, y, z, connected);
				//if(connected == -1) parentJoint = id;
				//joints.push_back(j);

			} else {
				if(id==-99){
					std::cout<<"finished reading skeleton data"<<std::endl;
				} else {
					std::cout<<"Skeleton not formatted correctly"<<std::endl;
					std::cout<<"ID: "<<id<<" SIZE: "<<bones.size()<<std::endl;
				}

			}
			
		}
		int numWeights = bones.size();
		ifstream weight_file(weights_file);
		if(weight_file==NULL) cerr << "failed reading weights data from " << weights_file;
		char buf2[1024];
		while(!weight_file.eof()) {
			weight_file.getline(buf2, sizeof(buf));
			Weight w(numWeights);
			for(int i = 0; i < numWeights; i++) {
				float wi;
				sscanf(buf, "%f", &wi);
				w.addWeight(wi, i);
			}
			weights.push_back(w);
		}
		/*
		for(int ji = 0; ji < joints.size(); ji++)
		{
			if(joints[ji].connected == -1) {
				// parent joint
			} else {
				Bone b(joints[joints[ji].connected], joints[ji]);
				bones.push_back(b);
			}
		}*/

		for(int p = 0; p < bones.size(); p++)
		{
			bones[p]->InitGlobal();

		}
		std::cout<<"Initialised bone matrices"<<std::endl;


	};





};






class Triangle {
friend class TriangleMesh;

	int _id;
	int _vertex[3];
	int _normal[3];
	int _edge[3];
	float _min, _max;
	float _color; 
	float _area;
	float _ratio;

public:

	Triangle(int v1, int v2, int v3, int n1, int n2, int n3) 
	{
		_vertex[0] = v1;  _vertex[1] = v2;  _vertex[2] = v3;  
		_normal[0] = n1;  _normal[1] = n2;  _normal[2] = n3;  
		
	};

	void setMorseMinMax(float min, float max)
	{
		_min = min; _max = max;
	}

	void getMorseMinMax(float & min, float & max)
	{
		min = _min; max = _max;
	}

	void setEdge(int e1, int e2, int e3)
	{
		_edge[0] = e1;
		_edge[1] = e2;
		_edge[2] = e3;
	};

	int edge(int i) { return _edge[i];};
	int id() { return _id;};
	void setColor(float f) { _color = f ;};
	float color() { return _color;};
};

class Edge {

	friend bool contain(Edge & e, map < pair <int, int> , Edge > & list) ;
	friend int edgeID(Edge & e, map < pair <int, int> , Edge > & list) ;
	friend bool contain(Edge & e, vector < Edge > & list) ;
	friend int edgeID(Edge & e, vector < Edge > & list) ;

	int _v1,_v2;
	vector <int> _trig_list;
	float length;
	int _id;

public: 

	Edge () {};

	Edge (int i, int j)
	{
		_v1 = i; _v2 = j;
	};

	bool operator == (Edge & e)
	{
		if (this->_v1 == e._v1 && this->_v2 == e._v2) return true;
		if (this->_v1 == e._v2 && this->_v2 == e._v1) return true;

		return false;
	}

	void add_triangle(int trig)
	{
		for (int i = 0; i < _trig_list.size(); i++) 
			if (trig == _trig_list[i]) return;

		_trig_list.push_back(trig);
	}

	void other_trig(const int trig, vector <int> & others) 
	{
		for (int i = 0; i < _trig_list.size(); i++) {
			if (_trig_list[i] == trig) continue; 
			else others.push_back(_trig_list[i]);
		}
	}


	void setId(int id) { _id = id;};
	int id() { return _id;};
	int v1() { return _v1;};
	int v2() { return _v2;};
	vector <int> getTrigList() { return _trig_list ;};
};




struct Node {
	int _id;

	vector<int> edges_to;     
	vector<float> edges_cost;   

	bool done;   
	float cost;    
};






float fmax(float f1,float f2, float f3) {
	float f = f1;

	if (f < f2) f = f2;
	if (f < f3) f = f3;

	return f;
};

float fmin(float f1,float f2, float f3) {
	float f = f1;

	if (f > f2) f = f2;
	if (f > f3) f = f3;

	return f;
};


class TriangleMesh 
{
	vector <Vector3f> _v;
	vector <Vector3f> _vn;
	vector <Triangle> _trig;
	vector <Node> _node;
	vector <Edge> _edge;

//	map <pair < int, int > , Edge > _edge;

	float _xmax, _xmin, _ymax, _ymin, _zmin, _zmax;



public: 
	TriangleMesh(char * filename) { loadFile(filename) ;};
	TriangleMesh() {};
	void loadFile(char * filename);

	int trigNum() { return _trig.size() ;};

	void getTriangleVertices(int i, Vector3f & v1, Vector3f & v2, Vector3f & v3)
	{
		v1 = _v[_trig[i]._vertex[0]]; 
		v2 = _v[_trig[i]._vertex[1]]; 
		v3 = _v[_trig[i]._vertex[2]]; 
	}

	void getTriangleIndices(int i, int& i1, int& i2, int& i3)
	{
		i1 = _trig[i]._vertex[0];
		i2 = _trig[i]._vertex[1];
		i3 = _trig[i]._vertex[2];
	}
	
	void getTriangleNormals(int i, Vector3f & v1, Vector3f & v2, Vector3f & v3)
	{
		v1 = _vn[_trig[i]._normal[0]]; 
		v2 = _vn[_trig[i]._normal[1]]; 
		v3 = _vn[_trig[i]._normal[2]]; 
	}
		

	void getMorseValue(int i, float & v1, float & v2, float & v3)
	{
		v1 = _node[_trig[i]._vertex[0]].cost; 
		v2 = _node[_trig[i]._vertex[1]].cost; 
		v3 = _node[_trig[i]._vertex[2]].cost; 
	}

	float color(int i) { return _trig[i].color();};


	void setMorseMinMax(int i, float min, float max)
	{
		_trig[i].setMorseMinMax(min,max);
	}

	void getMorseMinMax(int i, float & min, float & max)
	{
		_trig[i].getMorseMinMax(min,max);
	}


	void calcTriangleArea() 
	{
		Vector3f v1,v2,v3;

		for (int i = 0 ;i < _trig.size(); i++) 
		{
			getTriangleVertices(i, v1,v2,v3);
			v3 -= v1;
			v2 -= v1;

			_trig[i]._area = 0.5f*sqrt(v3.dot(v3)*v2.dot(v2) - (v3.dot(v2)*(v3.dot(v2))));  
//			cout << "trig " << i << " v2 " << v2 << " v3 " << v3 << " area = " << _trig[i]._area << endl;
		}
	}
};

#endif //_rt_H
