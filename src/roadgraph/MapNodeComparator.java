package roadgraph;

import java.util.Comparator;

public class MapNodeComparator implements Comparator<MapNode>{
	
	@Override
	public int compare(MapNode one , MapNode two)
	{
		if(one.distance > two.distance)
			return 1;
		else if(one.distance <two.distance)
			return -1;
		return 0;
		
	}
}
