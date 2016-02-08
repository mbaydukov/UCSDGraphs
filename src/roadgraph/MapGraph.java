/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
    private Map<GeographicPoint, MapNode> vertices;

    private long dijkstraCounter = 0l;
    private long aStarSearchCounter = 0l;

	/**
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
        this.vertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.keySet();
	}

    private MapNode getMapNode(GeographicPoint location)
    {
        return vertices.get(location);
    }

	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return vertices.values().stream().mapToInt(node -> node.getEdges().size()).sum();
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
        boolean success = true;
        if (location != null && !vertices.containsKey(location)){
            vertices.put(location, new MapNode(location));
        } else {
            success = false;
        }
		return success;
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
        if (from == null || to == null || roadName == null || roadType == null || length < 0
                || !getVertices().contains(from) || !getVertices().contains(to)){
            throw new IllegalArgumentException();
        } else {
            getMapNode(from).addEdge(new MapEdge(getMapNode(from), getMapNode(to), roadName, roadType, length));
        }
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

        //Initialize parameters
        Queue<MapNode> queue = new LinkedList<>();
        Set<MapNode> visitedNodes = new HashSet<>();
        HashMap<MapNode, Set<MapNode>> parent = new HashMap<>();

        queue.add(getMapNode(start));

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

        //Call recursive method
        bfs(goal, queue, visitedNodes, parent, nodeSearched);

        //Reconstruct found path
		return reconstructPath(start, goal, parent);
	}

    private void bfs(GeographicPoint goal, Queue<MapNode> queue, Set<MapNode> visitedNodes, Map<MapNode, Set<MapNode>> parent, Consumer<GeographicPoint> nodeSearched){
        if (!queue.isEmpty()) {
            MapNode currentNode = queue.poll();
            visitedNodes.add(currentNode);
            nodeSearched.accept(currentNode.getLocation());
            if (goal.equals(currentNode.getLocation())) {
                //Clear the queue in order to stop further search
                queue.clear();
            } else {
                //Find neighbours and add them to queue
                Set<MapNode> neighbors = currentNode.getNeighbours().stream().filter(node -> !visitedNodes.contains(node)).collect(Collectors.toSet());
                parent.put(currentNode, neighbors);
                queue.addAll(neighbors);
                bfs(goal, queue, visitedNodes, parent, nodeSearched);
            }
        }
    }


    /**
     * Returns path reconstructed from parent Map. Returns null if path was not found.
     * @param start GeographicPoint start location
     * @param goal GeographicPoint goal location
     * @param parent parent Map of MapNodes
     * @return
     */
    private List<GeographicPoint> reconstructPath(GeographicPoint start, GeographicPoint goal, Map<MapNode, Set<MapNode>> parent){
        List<GeographicPoint> path = null;

        boolean pathFound = parent.values().stream().flatMap(coll -> coll.stream()).collect(Collectors.toSet()).contains(getMapNode(goal));

        if (pathFound){
            path = new LinkedList<>();
            MapNode pathNode = getMapNode(goal);
            path.add(pathNode.getLocation());
            while (!pathNode.equals(getMapNode(start))) {
                MapNode currentNode = pathNode;
                pathNode = parent.entrySet().stream().filter(pNode -> pNode.getValue().contains(currentNode)).findFirst().get().getKey();
                path.add(pathNode.getLocation());
            }
            Collections.reverse(path);
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
		// TODO: Implement this method in WEEK 3

        //Initialize parameters
        PriorityQueue<NodeDistance> queue = new PriorityQueue<>(Comparator.<NodeDistance>reverseOrder());
        HashMap<NodeDistance, NodeDistance> visitedNodes = new HashMap<>();
        HashMap<MapNode, Set<MapNode>> parent = new HashMap<>();

        //Set distance for start node
        NodeDistance startNodeDistance = new NodeDistance(getMapNode(start), 0d);
        queue.add(startNodeDistance);
        visitedNodes.put(startNodeDistance, startNodeDistance);

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        //Call recursive method
        dijkstra(goal, queue, visitedNodes, parent, nodeSearched);

        //Reconstruct found path
        return reconstructPath(start, goal, parent);
	}

    /**
     * Recursive implementation of Dijkstra's algorithm
     * @param goal The target location
     * @param queue Priority queue
     * @param visitedNodes The set of visited nodes
     * @param parent The parent map of visited nodes
     * @param nodeSearched Consumer object for search visualization
     */
    private void dijkstra(GeographicPoint goal, PriorityQueue<NodeDistance> queue, Map<NodeDistance, NodeDistance> visitedNodes, Map<MapNode, Set<MapNode>> parent, Consumer<GeographicPoint> nodeSearched){
        if (!queue.isEmpty()) {
            NodeDistance currentNodeDistance = queue.poll();
            MapNode currentNode = currentNodeDistance.getMapNode();
            nodeSearched.accept(currentNode.getLocation());
            dijkstraCounter++;
            if (goal.equals(currentNode.getLocation())) {
                //Clear the queue in order to stop further search
                queue.clear();
            } else {
                //Get distance from start to current node
                double currentDistance = currentNodeDistance.getDistance();
                //Find neighbours and add them to queue
                Set<MapNode> neighbors = currentNode.getNeighbours();
                Iterator<MapNode> it = neighbors.iterator();
                while (it.hasNext()){
                    MapNode neighbor = it.next();
                    double calculatedDistance =  currentDistance + currentNode.getLocation().distance(neighbor.getLocation());
                    NodeDistance nodeDistance = new NodeDistance(neighbor, calculatedDistance);
                    if (visitedNodes.keySet().contains(nodeDistance) && !(visitedNodes.get(nodeDistance).getDistance() > nodeDistance.getDistance())){
                        it.remove();
                    } else {
                        queue.add(nodeDistance);
                        visitedNodes.put(nodeDistance, nodeDistance);
                        for (Set<MapNode> mapNodes: parent.values()){
                            mapNodes.remove(nodeDistance.getMapNode());
                        }
                    }
                }
                parent.put(currentNode, neighbors);
                dijkstra(goal, queue, visitedNodes, parent, nodeSearched);
            }
        }
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
        PriorityQueue<NodeDistance> queue = new PriorityQueue<>(Comparator.<NodeDistance>reverseOrder());
        HashMap<NodeDistance, NodeDistance> visitedNodes = new HashMap<>();
        HashMap<MapNode, Set<MapNode>> parent = new HashMap<>();

        NodeDistance startNodeDistance = new NodeDistance(getMapNode(start), start.distance(goal));
        queue.add(startNodeDistance);
        visitedNodes.put(startNodeDistance, startNodeDistance);

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
        aStarSearch(goal, queue, visitedNodes, parent, nodeSearched);

        //Reconstruct found path
        return reconstructPath(start, goal, parent);
	}


    /**
     * Recursive implementation of aStarSearch algorithm
     * @param goal The target location
     * @param queue Priority queue
     * @param visitedNodes The set of visited nodes
     * @param parent The parent map of visited nodes
     * @param nodeSearched Consumer object for search visualization
     */
    private void aStarSearch(GeographicPoint goal, PriorityQueue<NodeDistance> queue, Map<NodeDistance, NodeDistance> visitedNodes, Map<MapNode, Set<MapNode>> parent, Consumer<GeographicPoint> nodeSearched){
        if (!queue.isEmpty()) {
            NodeDistance currentNodeDistance = queue.poll();
            MapNode currentNode = currentNodeDistance.getMapNode();
            nodeSearched.accept(currentNode.getLocation());
            aStarSearchCounter++;
            if (goal.equals(currentNode.getLocation())) {
                //Clear the queue in order to stop further search
                queue.clear();
            } else {
                //Get distance from start to current node
                double currentDistance = currentNodeDistance.getDistance();
                //Find neighbours and add them to queue
                Set<MapNode> neighbors = currentNode.getNeighbours();
                Iterator<MapNode> it = neighbors.iterator();
                while (it.hasNext()){
                    MapNode neighbor = it.next();
                    double heuristicDistance =  currentDistance + currentNode.getLocation().distance(neighbor.getLocation()) + neighbor.getLocation().distance(goal);
                    NodeDistance nodeDistance = new NodeDistance(neighbor, heuristicDistance);
                    if (visitedNodes.keySet().contains(nodeDistance) && !(visitedNodes.get(nodeDistance).getDistance() > nodeDistance.getDistance())){
                        it.remove();
                    } else {
                        queue.add(nodeDistance);
                        visitedNodes.put(nodeDistance, nodeDistance);
                        for (Set<MapNode> mapNodes: parent.values()){
                            mapNodes.remove(nodeDistance.getMapNode());
                        }
                    }
                }
                parent.put(currentNode, neighbors);
                aStarSearch(goal, queue, visitedNodes, parent, nodeSearched);
            }
        }
    }

    public static void main(String[] args)
	{
		//System.out.print("Making a new map...");
		//MapGraph theMap = new MapGraph();
		//System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        //System.out.println();
        //System.out.println(theMap.aStarSearch(new GeographicPoint(1, 1), new GeographicPoint(8, -1)));
        //System.out.println(theMap.dijkstra(new GeographicPoint(1, 1), new GeographicPoint(8, -1)));
		//System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

        System.out.println(route);
        System.out.println(theMap.dijkstraCounter);

        System.out.println(route2);
        System.out.println(theMap.aStarSearchCounter);

	}
	
}
