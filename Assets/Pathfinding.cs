using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using Debug = UnityEngine.Debug;


public class Pathfinding : MonoBehaviour {

	//public Transform seeker, target;

	public Transform seekerAstar, targetAstar, seekerAEuc, targetAEuc, seekerAMan, targetAMan, seekerUCS, targetUCS,seekerBFS, targetBFS, seekerDFS, targetDFS;
	public Stopwatch timerAstar = new Stopwatch();
	public Stopwatch timerAEuc = new Stopwatch();
	public Stopwatch timerAMan = new Stopwatch();
	public Stopwatch timerUCS = new Stopwatch();
	public Stopwatch timerBFS = new Stopwatch();
	public Stopwatch timerDFS = new Stopwatch();
	public int totalMemoryUsedAstar = 0,  totalMemoryUsedUCS = 0, totalMemoryUsedBFS = 0, totalMemoryUsedDFS = 0, totalMemoryUsedAEuc = 0,totalMemoryUsedAMan=0;
	Grid2 grid;

	void Awake() {
		grid = GetComponent<Grid2> ();
	}

	void Update() {  // This function makes the script rund continuously.
/*	

		FindPathA (seekerAstar.position, targetAstar.position);
				Debug.Log("A* runtime:"+timerAstar.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAstar);

		FindPathAEuc (seekerAEuc.position, targetAMan.position);
				Debug.Log("A* with Euc heuristic runtime:"+timerAEuc.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAEuc);
		
		FindPathAMan (seekerAMan.position, targetAEuc.position);
				Debug.Log("A* with Euc heuristic runtime:"+timerAMan.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAMan);


		FindPathUCS (seekerUCS.position, targetUCS.position);
				Debug.Log("UCS runtime:"+timerUCS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedUCS);

		FindPathBFS (seekerBFS.position, targetBFS.position);
				Debug.Log("BFS runtime:"+timerBFS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedBFS);
 	
		FindPathDFS (seekerDFS.position, targetDFS.position);
			Debug.Log("DFS runtime:"+timerDFS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedDFS);
    
	    */
	}

	void Start() {  // The functions to run.
	
		 // prints the times once at after the completion of each algorithm
	    FindPathA (seekerAstar.position, targetAstar.position);
				Debug.Log("A*, video runtime:"+timerAstar.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAstar);

		FindPathAEuc (seekerAEuc.position, targetAMan.position);
				Debug.Log("A*, Euclidean runtime:"+timerAEuc.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAEuc);
		
		FindPathAMan (seekerAMan.position, targetAEuc.position);
				Debug.Log("A*, Manhattan runtime:"+timerAMan.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedAMan);
		
		FindPathUCS (seekerUCS.position, targetUCS.position);
				Debug.Log("UCS runtime:"+timerUCS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedUCS);

		FindPathBFS (seekerBFS.position, targetBFS.position);
				Debug.Log("BFS runtime:"+timerBFS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedBFS);
	/*
		FindPathDFS (seekerDFS.position, targetDFS.position);
			Debug.Log("DFS runtime:"+timerDFS.Elapsed.ToString()+"Memory usage:"+totalMemoryUsedDFS);
		*/
	}



	void FindPathA(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerAstar.Start();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePath(startNode,targetNode);
				timerAstar.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}
 

	void FindPathAEuc(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerAEuc.Start();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePathEuc(startNode,targetNode);
				timerAEuc.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceSQRT(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceSQRT(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

void FindPathAMan(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerAMan.Start();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				RetracePathMan(startNode,targetNode);
				timerAMan.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceManhattan(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceManhattan(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPathUCS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		timerUCS.Start();
		openSet.Add(startNode);
		
		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].gCost < node.gCost || openSet[i].gCost == node.gCost) {
					
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				
				RetracePathUCS(startNode,targetNode);
				timerUCS.Stop();
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		timerUCS.Stop();
	}


	void FindPathBFS(Vector3 startPos, Vector3 targetPos) { 
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
      Queue<Node> queueBFS = new Queue<Node>();
	  HashSet<Node> closedSet = new HashSet<Node>();
	  timerBFS.Start();
		queueBFS.Enqueue(startNode);
        
		while(queueBFS.Count!=0) { 
			Node currentNode= queueBFS.Dequeue();
			if (currentNode == targetNode) {
				RetracePathBFS(startNode,targetNode);
				timerBFS.Stop();
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !queueBFS.Contains(neighbour)) {
					closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					queueBFS.Enqueue(neighbour);
				}
		}
		}
		timerBFS.Stop();
	}


	void FindPathDFS(Vector3 startPos, Vector3 targetPos) { 
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
      Stack<Node> StackDFS = new Stack<Node>();
	  HashSet<Node> closedSet = new HashSet<Node>();
	    timerDFS.Start();
		StackDFS.Push(startNode);
        
		while(StackDFS.Count!=0) { 
			Node currentNode= StackDFS.Pop();
			if (currentNode == targetNode) {
				
				RetracePathDFS(startNode,targetNode);
				timerDFS.Stop();
				return;
			}
			closedSet.Add(currentNode);
		foreach (Node neighbour in grid.GetNeighbours(currentNode))
		{
			if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
			if (neighbour.walkable || !StackDFS.Contains(neighbour)) {
						//closedSet.Add(neighbour);
					neighbour.parent = currentNode; 
					StackDFS.Push(neighbour);
				}
		}
		}
		timerDFS.Stop();
	}



	void RetracePath(Node startNode, Node endNode) {
		
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedAstar = totalMemoryUsedAstar + 1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathA = path;

	}

	void RetracePathEuc(Node startNode, Node endNode) {
		
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedAEuc = totalMemoryUsedAEuc + 1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		grid.pathEuc = path;

	}

	void RetracePathMan(Node startNode, Node endNode) {
		
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedAMan = totalMemoryUsedAMan + 1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		grid.pathMan = path;

		}








	void RetracePathUCS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedUCS=totalMemoryUsedUCS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathUCS = path;

	}
    void RetracePathBFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedBFS=totalMemoryUsedBFS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathBFS = path;

	}
    void RetracePathDFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			totalMemoryUsedDFS=totalMemoryUsedDFS+1;
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		grid.pathDFS = path;

	}


	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}

	
	int GetDistanceSQRT(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		
		return (int) Mathf.Sqrt((Mathf.Pow(dstX, 2) + Mathf.Pow(dstY, 2))); 
		
	}
	int GetDistanceManhattan(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		return (dstY+dstX);
	}




}
