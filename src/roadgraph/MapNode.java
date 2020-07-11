package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

/*
 * Class used to represent Nodes
 * Each node is connected to a collection of edges
 * Each node represents a GeographicPoint*/
public class MapNode implements Comparable <MapNode>{
	private GeographicPoint location;
	private int numEdges;
	private List<MapEdge> edges;
	
	//Variable used in the Dijkstra algorithm
	//Distance from the node to the starting node
	private double distance;
	
	//Variable used in the A* algorithm
	//heuristic estimation
	private double function;
	
	//Newly created MapNode, not edges available yet
	public MapNode(GeographicPoint location)  {
		this.location = location;
		this.numEdges = 0;
		this.edges = new ArrayList<>();
		this.distance = 0;
		this.function = 0;
	}
	
	//Connection between two Nodes
	public void addNeighbor (MapNode start, MapNode end,String roadName, String roadType, double length) {
		this.edges.add(new MapEdge(roadName,roadType,length,start,end));
		this.numEdges++;	
	}
	
	public int getNumEdges() {
		return this.numEdges;
	}
	
	public List<MapEdge> getMapEdges() {
		return this.edges;
	}
	
	public GeographicPoint getLocation() {
		return this.location;
	}
	
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	public void setFunction(double function) {
		this.function = function;
	}
	
	public double getFunction() {
		return this.function;
	}
	
	public int compareTo(MapNode node) {
		if(this.function<node.function){
			return -1;
		} else if(this.function>node.function){
			return 1;
		}
		return 0;
	}
	
}
