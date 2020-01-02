
#include "HelpStudents.h"

#include <set>
#include<bits/stdc++.h>


using namespace std;

HelpStudents::HelpStudents(int  N, int  M, int K, vector < pair< pair <int,int> , int > > ways) {
    // IMPLEMENT ME!
    this->K = K;
    this->ways = ways;
    this-> N= N;
    this->M = M;
    adjacencyList.resize(N+1);
    for(int i=0; i<this->M;i++) {
         long long int x = (ways[i].first).first;
         long long int y = (ways[i].first).second;
        long long int weight = ways[i].second;
         adjacencyList[x].push_back(make_pair(y,weight));
         adjacencyList[y].push_back(make_pair(x,weight));
    }

}

long long int HelpStudents::firstStudent() {


    set<pair<long long int,long long int>,less<>> s;


    vector<long long int> dist(N+1,-1);
    s.insert(make_pair(0,1));
    dist[1]=0;
    while(!s.empty()) {

        pair<long long int, long long int> tmp = *(s.begin());
        s.erase(s.begin());
        long long int v = tmp.second;
        vector<pair<long long int,long long int>>::iterator i;
        for( i=adjacencyList[v].begin(); i!=adjacencyList[v].end();++i) {
            long long int node = (*i).first;
            long long  int weight = (*i).second;
            if(dist[node]>dist[v]+weight) {

                s.erase(s.find(make_pair(dist[node],node)));
                dist[node] = dist[v] + weight;
                s.insert(make_pair(dist[node], node));
            }
            else if(dist[node]<0) {
                dist[node] = dist[v]+weight;
                s.insert(make_pair(dist[node], node));
            }
        }
    }

    long long int a = dist[K];
    return a;


    // IMPLEMENT ME!
}
long long int HelpStudents::secondStudent() {

    set<pair<long long int,long long int>,less<>> s;
    vector<long long int> parent(N+1,-1);
    vector<bool> visited(N+1,false);
    vector<long long int> dist(N+1,-1);
    s.insert(make_pair(0,1));
    dist[1]=0;
    parent[1]=0;
    while(!s.empty()) {

        pair<long long int, long long int> tmp = *(s.begin());
        s.erase(s.begin());
        long long int v = tmp.second;
        visited[v]=true;
        vector<pair<long long int,long long int>>::iterator i;
        for( i=adjacencyList[v].begin(); i!=adjacencyList[v].end();++i) {
            long long int node = (*i).first;
            long long  int weight = (*i).second;
            if(!visited[node]) {
                if (dist[node] > weight) {
                    s.erase(s.find(make_pair(dist[node], node)));
                    dist[node] = weight;
                    s.insert(make_pair(dist[node], node));
                    parent[node] = v;
                } else if (dist[node] < 0) {
                    dist[node] = weight;
                    s.insert(make_pair(dist[node], node));
                    parent[node] = v;
                }
            }
        }
    }
    long long int max=0;
    long long int par = parent[K];
    long long int distance = dist[K];
    while(true) {
        if(par==0) {
            break;
        }
        if(distance>max) {
            max = distance;
        }

        distance = dist[par];
        par = parent[par];

    }

    return max;

    // IMPLEMENT ME!
}
long long int HelpStudents::thirdStudent() {
    set<pair<long long int,long long int>,less<>> s;
    vector<long long int> parent(N+1,-1);
    vector<bool> visited(N+1,false);
    vector<long long int> dist(N+1,-1);
    s.insert(make_pair(0,1));
    dist[1]=0;
    parent[1]=0;
    while(!s.empty()) {

        pair<long long int, long long int> tmp = *(s.begin());
        s.erase(s.begin());
        long long int v = tmp.second;
        visited[v]=true;
        vector<pair<long long int,long long int>>::iterator i;
        for( i=adjacencyList[v].begin(); i!=adjacencyList[v].end();++i) {
            long long int node = (*i).first;

            if(!visited[node]) {
                if (dist[node] > dist[v]+1) {
                    s.erase(s.find(make_pair(dist[node], node)));
                    dist[node] = dist[v]+1;
                    s.insert(make_pair(dist[node], node));
                    parent[node] = v;
                } else if (dist[node] < 0) {
                    dist[node] =dist[v]+1;
                    s.insert(make_pair(dist[node], node));
                    parent[node] = v;
                }
            }
        }
    }
return dist[K];

// IMPLEMENT ME!
}
long long int HelpStudents::fourthStudent() {
    long long int total = 0;
    long long int v = 1;
    while(true) {
    if(v==K) {
        break;
    }
        vector<pair<long long int,long long int>>::iterator i;
        long long int min = LONG_LONG_MAX;
        long long int minNode = 0;
        long long int index = -1;
        long long int minIndex = 0;
        if(adjacencyList[v].empty()) {
            return -1;
        }
        for( i=adjacencyList[v].begin(); i!=adjacencyList[v].end();++i) {
            index++;
            long long int node = (*i).first;
            long long  int weight = (*i).second;

                if (weight < min) {
                    min = weight;
                    minNode = node;
                    minIndex = index;
                } else if (weight == min) {
                    if (node < minNode) {
                        minNode = node;
                        minIndex = index;
                    }
                }
            }

        adjacencyList[v].erase(adjacencyList[v].begin()+minIndex);
        for( i=adjacencyList[minNode].begin(); i!=adjacencyList[minNode].end();++i) {
            long long int node = (*i).first;
            if(node==v) {
                adjacencyList[minNode].erase(i);
                break;
            }
        }
        v = minNode;
        total+=min;
        }
    return total;
    // IMPLEMENT ME!
}
long long int HelpStudents::fifthStudent() {

    set<pair<long long int,pair<long long int,long long int>>,less<>> s;
    vector<vector<long long int>> dist(N+1);
    vector<vector<long long int>> powCount(N+1);

    for(int i=0;i<N+1;i++) {
        dist[i].resize(3); powCount[i].resize(3);
        dist[i][0]=-1; powCount[i][0]=0;
        dist[i][1]=-1; powCount[i][1]=0;
        dist[i][2]=-1; powCount[i][2]=0;
    }
    dist[1][0]=0; powCount[1][0]=1;
    dist[1][1]=0;
    dist[1][2]=0;
    s.insert(make_pair(0,make_pair(1,0)));
    while(!s.empty()) {
        pair<long long int, long long int> tmp = (s.begin())->second;
        long long int dis = (s.begin())->first;
        long long int pow = tmp.second;

        s.erase(s.begin());
        long long int v = tmp.first;

        vector<pair<long long int,long long int>>::iterator i;
        for( i=adjacencyList[v].begin(); i!=adjacencyList[v].end();++i) {
            long long int node = (*i).first;
            long long int weight = (*i).second;
            if(pow==0) {
                if(weight>2*stepBack(v)) {
                    weight = 2*stepBack(v);
                }
            }
            if(pow ==2) {
                weight = 0;
            }
            long long int pow2=pow+1;
            if(pow==2) {
                pow2=0;
            }
             if(powCount[node][pow2]!=0) {
                 if (dist[node][pow2] > dis + weight) {

                     s.erase(s.find(make_pair(dist[node][pow2], make_pair(node,pow2))));
                     dist[node][pow2]= dis + weight;

                     s.insert(make_pair(dist[node][pow2], make_pair(node, pow2)));
                 } else if (dist[node][pow2] < 0) {
                     dist[node][pow2]= dis + weight;
                     powCount[node][pow2]=1;
                     s.insert(make_pair(dist[node][pow2], make_pair(node, pow2)));
                 }
             } else  {
                 dist[node][pow2]= dis + weight;
                 powCount[node][pow2]=1;
                 s.insert(make_pair(dist[node][pow2], make_pair(node, pow2)));
             }
        }

        }

    long long int min= LONG_LONG_MAX;
    for(int k =0; k<3; k++) {

        if(dist[K][k]<min && dist[K][k]!=-1) {
            min = dist[K][k];

        }
    }
    return min;

    // IMPLEMENT ME!
}
long long int  HelpStudents::stepBack(long long int node) {
    vector<pair<long long int,long long int>>::iterator i;
    long long int min = LONG_LONG_MAX;
    for( i=adjacencyList[node].begin(); i!=adjacencyList[node].end();++i) {
        if((*i).second < min) {
            min = (*i).second;

        }
    }
    return min;
}


// YOU CAN ADD YOUR HELPER FUNCTIONS
