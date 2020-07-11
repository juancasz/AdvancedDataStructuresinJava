package roadgraph;

public class MapEdge {
	private String roadName;
	private String roadType;
	private double length;
	private MapNode start;
	private MapNode end;
	
	//Edge between two nodes
	public MapEdge(String roadName,String roadType,double length,MapNode start,MapNode end) {
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
		this.start = start;
		this.end = end;
	}
	
	public void setLength(double length) {
		this.length = length;
	}
	
	public MapNode getStart() {
		return this.start;
	}
	
	public MapNode getEnd() {
		return this.end;
	}
	
	public double getDistance() {
		return this.length;
	}
	
}
