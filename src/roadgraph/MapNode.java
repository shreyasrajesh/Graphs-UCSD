package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	GeographicPoint vertex;									//Location of the vertex.
	List<MapEdge> edges = new ArrayList<MapEdge>();			//List of edges emerging from that vertex.
	double distance;										//Dijkstra's algorithm distance.
	double projdist;										//Projected distance for AStar algorithm.
	
	//Constructor
	public MapNode(){
		vertex = new GeographicPoint(10,10);
		edges = new ArrayList<MapEdge>();
		distance = Double.POSITIVE_INFINITY;
		projdist = Double.POSITIVE_INFINITY;
	}
	
	//Creating a new MapNode
	public MapNode(GeographicPoint vertex, List<MapEdge> edges){
		vertex = this.vertex;
		edges = this.edges;
		distance = Double.POSITIVE_INFINITY;
		projdist = Double.POSITIVE_INFINITY;
	}
	
	public MapNode(GeographicPoint vertex, double distance)
	{
		vertex = this.vertex;
		edges = new ArrayList<MapEdge>();
		distance = this.distance;
		projdist = Double.POSITIVE_INFINITY;
	}
	
	public MapNode(GeographicPoint vertex, double distance, double projdist)
	{
		vertex = this.vertex;
		edges = new ArrayList<MapEdge>();
		distance = this.distance;
		projdist = this.projdist;
	}
	
	public double DistanceFrom(MapNode other)
	{
		double thisx = this.vertex.getX();
		double thisy = this.vertex.getY();
		double otherx = other.vertex.getX();
		double othery = other.vertex.getY();
		double distance = Math.sqrt((thisx - otherx)*(thisx - otherx) + (thisy - othery)*(thisy - othery));
		return distance;
	}

}
