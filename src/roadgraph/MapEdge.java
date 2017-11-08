package roadgraph;

import geography.GeographicPoint;

public class MapEdge {

	GeographicPoint start;
	GeographicPoint end;
	String VertexName;
	String vertexType;
	double length;
	
	//Constructor
	public MapEdge() {
		
		start = new GeographicPoint(10,10);
		end = new GeographicPoint(10,10);
		VertexName = "abc";      					// Random values initialised which could be helpful in debugging.
		vertexType = "def";
		length = 0;
		}
	
	//Create a new MapEdge
	public MapEdge(GeographicPoint start,GeographicPoint end,String VertexName,String vertexType,double length)
	{
		vertexType = this.vertexType;
		VertexName = this.VertexName;
		start = this.start;
		end = this.end;
		length = this.length;
	}
	
	//Constructor for defining location without name(Just given for debugging with my own map file.)
	public MapEdge(GeographicPoint start,GeographicPoint end)
	{
		start = this.start;
		end = this.end;
	}

}
