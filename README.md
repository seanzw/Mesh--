# Mesh--
Simplify a triangle mesh using Quadratic Error Metric.

### How to use it.
After compiling it, type the following command in the command line and it will dump the simplified mesh as `output.obj`.
```
> Mesh--.exe <input.obj> <output.obj> <simplify_ratio>
```

### Data Structure
#####Vector
Vector is used to represent vertex and calculate quadratic error metric, thus its dimension can be 3 or 4. Inside it uses a double array with size 4.

#####Matrix
Matrix represents 4x4 matrix and is used to calculate quadratic error metric.

#####Edge
An edge is a struct of the id of its two end points v1 and v2. I have also defined the `<` operator for Edge so that I can store them in a `std::set`.
```
struct Edge {
	int v1;
	int v2;
	Edge(int v1 = -1, int v2 = -1) : v1(v1), v2(v2) {}
}
```

#####CrossLink
Since we are using edge collapsing algorithm, we have to store all the edges somewhere so that we can iterate through them, build the heap, e.t.c. At first I used `std::set` to store all the edges. However `std::set` is implemented as binary searching tree, which means it takes O(#Elg(#E)) to insert, delete and find an edge. To improve this, I implemented CrossLink to store them. Suppose an edge with its two end points v1 and v2, then its coordinates inside the CrossLink will be (v1, v2). Notice that to make every edge unique, v1 < v2 must be satisfied.

#####Mesh
`Mesh` uses data structures described above to represent a triangle mesh.
- It stores all the vertexes as 3 dimension `Vector` in `std::vector`.
- All the edges (v1, v2) with v1 < v2 are stored in CrossLink.
- It stores all the neighoring faces of a vertex in `std::vector<std::set<Edge>>`.
- It uses a `std::vector<bool>` to record whether a specific vertex has been removed.

The way `Mesh` storing neighboring faces to a vertex is a little complicated and here is a simple illustration.
<img href="img/illustrate.png"/>

Suppose the order of the three triangle is (v0, v2, v1), (v0, v1, v3), (v0, v3, v2). Then the neighboring faces of v0 is stored as
```
{
	{v2, v1},
	{v1, v3},
	{v3, v2}
}
```
Notice that the order of the other two vertexes matters because it represents the orientation of this triangle.

###Algorithm
To simplify a mesh, the program does the following procedures:
1. Calcuates the Quadratic Error Metric for every edge, which is the cost of collapsing this edge, and pushed them into a priority queue with -QEM as their priority.
2. Pops an edge from the queue and collapses it.
3. Recalculates QEM for all the neighboring edges that are affected by Step 2 and pushes them into the queue.
4. If terminal condition are satisified, save the simplified mesh. Otherwise, jump back to Step 2.

###Preserve the orientation.
The main problem in the previous simple algorithm is that it doesn't preserve the orientation of a triangle. This is illustrated as
<img href="img/orientation.png"/>

After collapsing edge (v1, v2) to new_v1, the orientation of the red triangle is changed, which is a disaster to this mesh. Thus I added a function to check that whether a face would be reversed after collapsing this edge and handle it by reverse it first.

###Result.
Here are some results from this program. I simplified them first with Mesh-- and then rendered them with my ray tracer engine [yart-cpp](https://github.com/seanzw/yart-cpp).



