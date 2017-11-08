/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import javax.xml.bind.JAXBElement.GlobalScope;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections ,between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	/** 
	 * Create a new empty MapGraph 
	 */
	// HashMap containing the location and the mapNode(i.e. class with details of that location)
	HashMap<GeographicPoint, MapNode> vertices = new HashMap<GeographicPoint, MapNode>();
	
	// List of MapEdges(Used initially and not used in the final implementation. Helpful in keeping track of the edges and debugging)
	//List<MapEdge> edges = new ArrayList<MapEdge>();
		
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		vertices = new HashMap<GeographicPoint, MapNode>();
		//edges = new ArrayList<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		int NumVertices=0;
		if(!(vertices.isEmpty()))
				NumVertices = vertices.size();
		return NumVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> locations = new HashSet<GeographicPoint>();
		Collection<MapNode> nodes = new ArrayList<MapNode>();
		nodes = vertices.values();
		for(Iterator<MapNode> iterator = nodes.iterator(); iterator.hasNext();)
		{
			locations.add((iterator.next()).vertex);
		}
		return locations;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		Collection<MapNode> nodes = new ArrayList<MapNode>();
		nodes = vertices.values();
		int count = 0;
		for(Iterator<MapNode> iterator = nodes.iterator(); iterator.hasNext();)
		{
			count = count + ((iterator.next()).edges).size();
		}
		int numEdges = count; 
		
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(!(vertices.containsKey(location) || location == null))
		{
			MapNode node = new MapNode();
			node.vertex = location;
			vertices.put(location, node);
		}
		return true;
	}
	
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		MapEdge edge = new MapEdge();
		edge.start = from;
		edge.end = to;
		edge.VertexName = roadName;
		edge.vertexType = roadType;
		edge.length = length;
		MapNode node = vertices.get(from);
		//System.out.println(node.edges);
		(node.edges).add(edge);							//Adds all the edges from that node to the edges list defined in MapNode
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		//Initializations(Step 1)
		MapNode S = new MapNode();
		MapNode G = new MapNode();
		S = vertices.get(start);
		G = vertices.get(goal);
		
		Set<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		
		//Enqueue S (Step 2)
		toExplore.add(S);
		boolean found = false;					// Variable to determine whether the goal has been reached or not.
		
		//Step 3
		while(!(toExplore.isEmpty())) {
			//Dequeue the first node.(Step 4)
			MapNode current = toExplore.remove();
			//System.out.println(current.vertex);
			//Break if the goal is reached. (Step 5)
			if((current.vertex).equals(G.vertex)) {
				found = true;
				break;
			}
			
			List<MapNode> neighbors = getNeighbors(current.vertex);
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			//For each new Neighbor for the vertex. (Step 6)
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				//System.out.println(next.vertex);
				if (!visited.contains(next)) {
					// Add the latest node to the visited set.(Step 7)
					nodeSearched.accept(next.vertex);
					visited.add(next);
					//Add the current node as the parent of the latest node. (Step 8)
					parentMap.put(next,current);
					//System.out.println(parentMap.get(next).vertex);
					//Enqueue the latest node. (Step 9)
					toExplore.add(next);
				}
			}				
  		}

			if(!found){
				System.out.println("No Path exists.");
				return new ArrayList<GeographicPoint>();
			}
			
			//reconstruct the path
			
			LinkedList<MapNode> path = new LinkedList<MapNode>();
			MapNode current = G;
			
			List<GeographicPoint> points = new LinkedList<GeographicPoint>();
			path.addFirst(G);
			current = parentMap.get(G);	
			// Loop to obtain the path starting from the end working our way backwards.
			while(current != S) {
				//System.out.println(current.vertex);
				path.addFirst(current);
				//points.add(current.vertex);
				current = parentMap.get(current);
				//System.out.println(current); 
			}
			path.addFirst(S);
			// Loop to store the path in the form of a list of GeographicPoint objects.
			for (MapNode i : path )
			{
				points.add(i.vertex);
			}
			
			return points;		
	}

	// GetNeighbors method that deals with determining the Neighbors of each of the vertices of the map. Helper method for the bfs method.
	public List<MapNode> getNeighbors(GeographicPoint point)
	{
		List<MapEdge> neighbors = vertices.get(point).edges;
		//System.out.println(neighbors.get(0).end);
		//System.out.println(current.edges.get(0).end);
		List<MapNode> neighbor_list = new ArrayList<MapNode>();
		for(Iterator<MapEdge> iterator = neighbors.iterator(); iterator.hasNext();)
		{
			GeographicPoint location = (iterator.next()).end;
			neighbor_list.add(vertices.get(location));
		}
		return neighbor_list;	
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		//Initializations(Step 1)
		MapNode S = new MapNode();
		MapNode G = new MapNode();
		S = vertices.get(start);
		G = vertices.get(goal);
		S.distance = 0;
		
		
		Set<MapNode> visited = new HashSet<MapNode>();
		Comparator<MapNode> dist = new MapNodeComparator();
		Queue<MapNode> toExplore = new PriorityQueue<MapNode>(10,dist);
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		int visitedDijkstra = 0;
		
		//Enqueue S (Step 2)
		toExplore.add(S);
		boolean found = false;					// Variable to determine whether the goal has been reached or not.
		
		//Step 3
		while(!(toExplore.isEmpty())) {
			//Dequeue the first node.(Step 4)
			MapNode current = toExplore.remove();
			//System.out.println(current.vertex);
			visitedDijkstra +=1;
			//Extra step
			if(!visited.contains(current))
			{
				visited.add(current);
			}
			//Break if the goal is reached. (Step 5)
			if((current.vertex).equals(G.vertex)) {
				found = true;
				break;
			}
			
			List<MapNode> neighbors = getNeighbors(current.vertex);
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			//For each new Neighbor for the vertex. (Step 6)
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				//System.out.println(next.vertex);
				if (!visited.contains(next)) {
					// Add the latest node to the visited set.(Step 7)
					double total_distance = current.distance + current.DistanceFrom(next);
					if(total_distance<next.distance)
					{
						
						// Hook for visualization.  See write up.
						nodeSearched.accept(next.vertex);
						parentMap.put(next,current);
						next.distance = total_distance;
						toExplore.add(next);
					}
					//System.out.println(parentMap.get(next).vertex);
					//Enqueue the latest node. (Step 9)
				}
			}				
  		}

			if(!found){
				System.out.println("No Path exists.");
				return new ArrayList<GeographicPoint>();
			}
			
			//reconstruct the path
			
			LinkedList<MapNode> path = new LinkedList<MapNode>();
			MapNode current = G;
			
			List<GeographicPoint> points = new LinkedList<GeographicPoint>();
			path.addFirst(G);
			current = parentMap.get(G);	
			// Loop to obtain the path starting from the end working our way backwards.
			while(current != S) {
				//System.out.println(current.vertex);
				path.addFirst(current);
				//points.add(current.vertex);
				current = parentMap.get(current);
				//System.out.println(current); 
			}
			path.addFirst(S);
			// Loop to store the path in the form of a list of GeographicPoint objects.
			for (MapNode i : path )
			{
				points.add(i.vertex);
			}
			System.out.println(visitedDijkstra);
			return points;		
				
		
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		//Initializations(Step 1)
		MapNode S = new MapNode();
		MapNode G = new MapNode();
		S = vertices.get(start);
		G = vertices.get(goal);
		S.distance = 0;
		S.projdist = 0;
		
		
		Set<MapNode> visited = new HashSet<MapNode>();
		Comparator<MapNode> dist = new MapNodeComparator();
		Queue<MapNode> toExplore = new PriorityQueue<MapNode>(10,dist);
		Map<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		int visitedAstar = 0;
		//Enqueue S (Step 2)
		toExplore.add(S);
		boolean found = false;	// Variable to determine whether the goal has been reached or not.
		
		//Step 3
		while(!(toExplore.isEmpty())) {
			//Dequeue the first node.(Step 4)
			MapNode current = toExplore.remove();
			visitedAstar +=1;
			//System.out.println(current.vertex);
			
			//System.out.println("Node at location " + current.vertex+ " with distance " + current.distance + " and " + current.projdist+"\n");
			//Extra step
			if(!visited.contains(current))
			{
				visited.add(current);
			}
			//Break if the goal is reached. (Step 5)
			if((current.vertex).equals(G.vertex)) {
				found = true;
				break;
			}
			
			List<MapNode> neighbors = getNeighbors(current.vertex);
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			//For each new Neighbor for the vertex. (Step 6)
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				//System.out.println(next.vertex);
				if (!visited.contains(next)) {
					// Add the latest node to the visited set.(Step 7)
					double dijkstra_distance = current.distance + next.DistanceFrom(current);
					double total_distance = dijkstra_distance + next.DistanceFrom(G);
					if(total_distance <  next.projdist)
					{	
						// Hook for visualization.  See write up.
						nodeSearched.accept(next.vertex);
						next.distance = dijkstra_distance;
						next.projdist = total_distance;
						parentMap.put(next,current);						
						toExplore.add(next);
					}
					//System.out.println(parentMap.get(next).vertex);
					//Enqueue the latest node. (Step 9)
				}
			}				
  		}

			if(!found){
				System.out.println("No Path exists.");
				return new ArrayList<GeographicPoint>();
			}
			
			//reconstruct the path
			
			LinkedList<MapNode> path = new LinkedList<MapNode>();
			MapNode current = G;
			
			List<GeographicPoint> points = new LinkedList<GeographicPoint>();
			path.addFirst(G);
			current = parentMap.get(G);	
			// Loop to obtain the path starting from the end working our way backwards.
			while(current != S) {
				//System.out.println(current.vertex);
				path.addFirst(current);
				//points.add(current.vertex);
				current = parentMap.get(current);
				//System.out.println(current); 
			}
			path.addFirst(S);
			// Loop to store the path in the form of a list of GeographicPoint objects.
			for (MapNode i : path )
			{
				points.add(i.vertex);
			}
			
			System.out.println(visitedAstar);
			
			return points;
	}

	
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//		List<GeographicPoint> path = firstMap.aStarSearch(testStart, testEnd);
//		System.out.println(path);
//		System.out.println("DONE.");
//		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
	}
	
}
