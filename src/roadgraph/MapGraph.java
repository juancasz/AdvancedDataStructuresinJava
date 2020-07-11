/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import org.w3c.dom.Node;

import geography.GeographicPoint;
import util.GraphLoader;
import java.util.Map;
import java.util.Queue;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private Map<GeographicPoint,MapNode> graph;
	private int numNodes;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		this.graph = new HashMap<>();
		this.numNodes = 0;
		this.numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return this.numNodes;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return this.graph.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return this.numEdges;
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
		// TODO: Implement this method in WEEK 3
		if(!(this.graph.containsKey(location))) {
			this.graph.put(location, new MapNode(location));
			this.numNodes++;
			return true;
		}
		return false;
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

		//TODO: Implement this method in WEEK 3
		if(!this.graph.containsKey(from)||!this.graph.containsKey(to)||length<0||roadType==null) {
			throw new IllegalArgumentException();
		}
		
		this.graph.get(from).addNeighbor(this.graph.get(from), this.graph.get(to), roadName, roadType, length);
		this.numEdges++; 	
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
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//the method will look between a start node and a goal node
		MapNode startNode = this.graph.get(start);
		MapNode goalNode = this.graph.get(goal);
		Map<MapNode,MapNode> parentMap = new HashMap<>(); 
		
		//check if the search was successful
		boolean successful = bfsSuccessful(startNode,goalNode,parentMap,nodeSearched);
		if(!successful) {
			return null;
		}
		
		return buildPath(start,goal,parentMap);
	}
	
	private boolean bfsSuccessful(MapNode startNode,MapNode goalNode,Map<MapNode,MapNode> parentMap,Consumer<GeographicPoint> nodeSearched) {
		Queue<MapNode> queue = new LinkedList<MapNode>();
		Set<MapNode> visited = new HashSet<MapNode>();
		
		queue.add(startNode);
		visited.add(startNode);
		
		
		//Breadth First Search working
		while(!queue.isEmpty()) {
			MapNode element = queue.poll();
			
			// Hook for visualization. 
			nodeSearched.accept(element.getLocation());
			
			if(element.equals(goalNode)) {
				return true;
			}
			
			for(MapEdge link: element.getMapEdges()) {
				MapNode neighbor = link.getEnd();
				if(!visited.contains(neighbor)) {
					queue.add(neighbor);
					visited.add(neighbor);
					parentMap.put(neighbor, element);
				}
			}
		}
		return false;
	}
	
	//Recover the path of nodes used to reach the goal 
	private List<GeographicPoint> buildPath(GeographicPoint start,GeographicPoint goal, Map<MapNode,MapNode> parentMap){
		
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		path.addFirst(goal);
		MapNode item = this.graph.get(goal);
		
		while(parentMap.containsKey(item)) {
			MapNode element = parentMap.get(item);
			path.addFirst(element.getLocation());
			item = element;
		}
		return path;
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
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//Set to infinity the distance of each node to the starting node
		for(MapNode node:this.graph.values()) {
			node.setDistance(Double.MAX_VALUE);
		}
		
		//Search the path between these two nodes
		MapNode startNode = this.graph.get(start);
		MapNode goalNode = this.graph.get(goal);
		
		//Parent HashMap and visited HashSet
		Map<MapNode,MapNode> parentMap = new HashMap<>();
		Set<MapNode> visited = new HashSet<MapNode>();
				
		
		//Distance from the starting node to the starting node is zero
		startNode.setDistance(0.0);
		
		//Priority queue
		//Queue <MapNode> queue = new LinkedList<>();
		Queue <MapNode> queue = new LinkedList<>();
		
		//Enqueue{S, 0} onto the PQ
		queue.add(startNode);
		
		//DIJKSTRA
		while(!queue.isEmpty()) {
			MapNode curr = queue.poll();
			
			//sent location to the GUI
			nodeSearched.accept(curr.getLocation());
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.getLocation().equals(goal)) {
					System.out.println(visited.size());
					return buildPath(start,goal,parentMap);
				}
				for(MapEdge link:curr.getMapEdges()) {
					MapNode neighbor = link.getEnd();
					double distance = curr.getDistance() + link.getDistance();
					if(distance<neighbor.getDistance()) {
						neighbor.setDistance(distance);
						parentMap.put(neighbor, curr);
						queue.add(neighbor);
					}
				}
			}
		}
		return null;
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//Search the path between these two nodes
		MapNode startNode = this.graph.get(start);
		MapNode goalNode = this.graph.get(goal);
		
		//Set to infinity the distance of each node to the starting node
		for(MapNode node:this.graph.values()) {
			node.setDistance(Double.MAX_VALUE);
			node.setFunction(Double.MAX_VALUE);
		}
				
				
		//Parent HashMap and visited HashSet
		Map<MapNode,MapNode> parentMap = new HashMap<>();
		//Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		List<GeographicPoint> visited = new ArrayList<GeographicPoint>();
		//Priority queue
		//Queue <MapNode> queue = new LinkedList<>();
		PriorityQueue <MapNode> queue = new PriorityQueue<MapNode>();
		
		//Distance from the starting node to the starting node is zero
		startNode.setDistance(0.0);
		startNode.setFunction(0.0);
				
		//Enqueue{S, 0} onto the PQ
		queue.add(startNode);
				
		//A*
		while(!queue.isEmpty()) {
			MapNode curr = queue.poll();
			//System.out.println(curr.getLocation());
			//sent location to the GUI
			nodeSearched.accept(curr.getLocation());
					
			if(!visited.contains(curr.getLocation())) {
				//System.out.println(curr.getLocation());
				visited.add(curr.getLocation());
				if(curr.getLocation().equals(goal)) {
					System.out.println(visited.size());
					return buildPath(start,goal,parentMap);
					//List<GeographicPoint> list = new ArrayList<GeographicPoint>(visited);
					//return list;
				}
				for(MapEdge link:curr.getMapEdges()) {
					MapNode neighbor = link.getEnd();
					double distance = curr.getDistance() + link.getDistance(); //g(n)
					double function = neighbor.getLocation().distance(goal)+distance; //f(n) = h(n)+g(n)
					neighbor.setDistance(distance);
					neighbor.setFunction(function);
					if(!visited.contains(neighbor.getLocation())){
						parentMap.put(neighbor, curr);
					}
					queue.add(neighbor);
					}
				}				
			}
		return null;
	}
	
	
	
	
	public static void main(String[] args)
	{	
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		System.out.println(firstMap.bfs(testStart, testEnd));
		*/
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println(testroute);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute2);
		*/
		
		/*
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
		*/
		
		
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
