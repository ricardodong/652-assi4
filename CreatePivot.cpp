#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstring> 
#include <cstdlib>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>
#include <map>

#include<windows.h>

using namespace std;

int transInt(char *CNum){
	// we assume that there is no . in number
	int result = 0;
	int length = strlen(CNum);
	for(int i=0; i < length; i++){
		result += CNum[i] - 48;
		result *= 10;
	}
	result/=10;
	return result;
}

class voxelNode{
private:
	vector<int> state;
	map<vector<int>, bool> *mapInfo;
	// 注意这是个指针！！ 
	int* mapBound;
	double gcost;
public:
	voxelNode(){}
	voxelNode(vector<int> coor, map<vector<int>, bool> *mapIn, int* boundary, double newGcost){
		state = coor;
		mapInfo = mapIn;
		mapBound = boundary;
		gcost = newGcost;
	}
	
	double getGcost(){
	    return gcost;
	}
	
	void setGcost(double a){
		gcost = a;
	}
	
	vector<int> getState(){
	    return state;
	}
	
	bool operator>(const voxelNode &b) const{
		return this->gcost > b.gcost;
	}
	
	bool operator==(const voxelNode &b) const{
		return this->gcost == b.gcost;
	}
	
	bool operator<(const voxelNode &b) const{
		return this->gcost < b.gcost;
	}
	
	short setNeigh(vector<voxelNode>& neighVec){
		short numNeigh = 0;
		
		vector<vector<vector<bool> > > blockSituation(3, vector<vector<bool> >(3, vector<bool>(3, false)));
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					vector<int> newState(state);
					newState[0] += i-1; newState[1] += j-1; newState[2] += k-1;
					if((*mapInfo)[newState]){
						blockSituation[i][j][k] = true;
						//cout<<"blocked?"<<endl;
					}
					if(newState[0]<0||newState[1]<0||newState[2]<0||newState[0]>mapBound[0]||newState[1]>mapBound[1]||newState[2]>mapBound[2]){
						blockSituation[i][j][k] = true;
						//cout<<"out?"<<newState[0]<<" "<<newState[1]<<" "<<newState[2]<<endl;
					}
				}
			}
		}
		
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					if(i==1&&j==1&&k==1) {continue;}
					
					if(i!=1){
						if(blockSituation[i][1][1]) {
						    //cout<<"i"<<endl;
						    continue;}
					}
					if(j!=1){
						if(blockSituation[1][j][1]) {
						    //cout<<"j"<<endl;
						    continue;}
					}
					if(k!=1){
						if(blockSituation[1][1][k]) {
						    //cout<<"k"<<endl;
						    continue;}
					}
					
					if(i!=1&&j!=1){
						if(blockSituation[i][j][1]) {
						    //cout<<"ij"<<endl;
						    continue;}
					}
					if(i!=1&&k!=1){
						if(blockSituation[i][1][k]) {
						    //cout<<"ik"<<endl;
						    continue;}
					}
					if(j!=1&&k!=1){
						if(blockSituation[1][j][k]) {
						    //cout<<"jk"<<endl;
						    continue;}
					}
					
					if(i!=1&&j!=1&&k!=1){
						if(blockSituation[i][j][k]) {
						    //cout<<"ijk"<<endl;
						    continue;}
					}
					
					vector<int> newState(state);
                    newState[0] += i-1; newState[1] += j-1; newState[2] += k-1;
                    double nextgcost = sqrt((i-1)*(i-1)+(j-1)*(j-1)+(k-1)*(k-1));
                    voxelNode newvonode(newState, this->mapInfo, this->mapBound, this->gcost+nextgcost);
                    neighVec[numNeigh] = newvonode;
                    numNeigh++;
				}
			}
		}
		//cout<<numNeigh<<" real: "<<neighVec.size()<<endl;
		return numNeigh;
	}
	
};

bool cmp(voxelNode &a, voxelNode &b){
	return a.getGcost()>b.getGcost();
}

class BFS{
private:
	vector<voxelNode> neighCache;
	double*** vomapGcost;
public:
	BFS(){}
	BFS(int* mapBound){
		voxelNode empty;
		for (int i = 0;i < 26;i++){
		    neighCache.push_back(empty);
		}
		
		vomapGcost = new double ** [mapBound[0]];
		for (int i = 0;i < mapBound[0];i++){
		    vomapGcost[i] = new double*[mapBound[1]];
		    for (int j = 0;j < mapBound[1];j++){
		        vomapGcost[i][j] = new double [mapBound[2]];
		    }
		}
		
		for(int i=0;i<mapBound[0];i++){
			for(int j=0;j<mapBound[1];j++){
				for(int k=0;k<mapBound[2];k++){
				    vomapGcost[i][j][k] = 0;
				}
			}
		}
	}
	
	
	void search(voxelNode start){
		vector<voxelNode> openList;
		openList.push_back(start);
		make_heap(openList.begin(), openList.end(), cmp);
		double frontGcost;
		voxelNode currFront;
		vector<int> frontState;
		short neighNum;
		bool needToHeapAgain;
		bool in_open;
		vector<voxelNode>::iterator it;
		int count = 0;
		
		while(!openList.empty()){
			count++;
			if(count%1000 == 0)cout<<count<<endl;
			currFront = openList.front();
			pop_heap(openList.begin(), openList.end(), cmp);
			openList.pop_back();
			
			frontState = currFront.getState();
			frontGcost = vomapGcost[frontState[0]][frontState[1]][frontState[2]];
			//vomapGcost 相当于closelist, 这里判断它不在closelist才expand 
			if (frontGcost == 0.0){
				vomapGcost[frontState[0]][frontState[1]][frontState[2]] = currFront.getGcost();
				//cout<<frontState[0]<<" "<<frontState[1]<<" "<<frontState[2]<<" "<<openList.size()<<endl;
				
		        neighNum = currFront.setNeigh(neighCache);
		        needToHeapAgain = false;
		        for(short m = 0;m<neighNum;m++){
		        	in_open = false;
		            for(it=openList.begin();it!=openList.end();++it){
		            	if(it->getState() == neighCache[m].getState()){
		            		in_open = true;
		            		if(neighCache[m].getGcost() < it->getGcost()){
							    it->setGcost(neighCache[m].getGcost()); 
								needToHeapAgain=true;
							}
		            		break;
						}
					}
					if(!in_open){
						openList.push_back(neighCache[m]);
						push_heap(openList.begin(), openList.end(), cmp);
					}
				}
				if(needToHeapAgain) {make_heap(openList.begin(), openList.end(), cmp);}
				
			}
			else{
				//frontGcost should not be bigger than currFront.gcost 
				//if(frontGcost > currFront.getGcost() ) cout<<"wrong!!!"<<endl;
			}
		}
		cout<<"end"<<endl;
	}
};

class voxelmap{
public:
	int mapinfo[3];
	map<vector<int>, bool> blockNodes;
	// I don't really need to use map to store blockNodes,
	// I can also use a 3D bool array to store it.
	BFS *pivotsInfo = new BFS[10];
	// 存在堆中保证空间够大 
	
	voxelmap(char *mapfileName){
		ifstream mapfile;
		mapfile.open(mapfileName);
		
		char readLine[256];
		mapfile.getline(readLine, 100);
		char *CMapinfo = NULL;
        CMapinfo = strtok(readLine, " ");
        CMapinfo = strtok(NULL," ");
        for(int i=0;i<3;++i){
        	mapinfo[i] = transInt(CMapinfo);
        	CMapinfo = strtok(NULL," ");
		}
		
        while (!mapfile.eof()){
        	mapfile.getline(readLine, 100);
            if(strlen(readLine) == 0){break;}
            char *CBlcokNode = strtok(readLine, " ");
            vector<int> blockCoor(3);
            for(int i=0;i<3;++i){
        	    blockCoor[i] = transInt(CBlcokNode);
        	    CMapinfo = strtok(NULL," ");
		    }
            blockNodes.insert(map<vector<int>, bool>::value_type(blockCoor, true));
		}
	}

    void CreateRandPivot(){
    	for(int i=0;i<10;i++){
    		Sleep(100);
    		srand((unsigned)time(0));
    		vector<int> randNum(3);
    		for(int j=0;j<3;j++){
    			randNum[j] = rand() % mapinfo[j];
    			cout<<randNum[j]<<" ";
			}
			cout<<endl;
			voxelNode randPivot(randNum, &blockNodes, mapinfo, 0.0);
			BFS newBFS(mapinfo);
            newBFS.search(randPivot);
            pivotsInfo[i] = newBFS; 
            cout<<"created one pivot"<<endl;
		}
	}
};

int main(){
	char *filename = "warframe/Simple.3dmap"; 
	voxelmap *newMap = new voxelmap(filename);
	cout<<"fine"<<endl;
	//newMap->CreateRandPivot();
	/*
	vector<int> test1(3, 1);
	vector<int> test2(3, 50);
	cout<<newMap->blockNodes[test1]<<" "<<newMap->blockNodes[test2]<<endl;
	*/
	return 0;
}
