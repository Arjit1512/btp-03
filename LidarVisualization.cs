/*
using UnityEngine;
using System.Collections.Generic;

public class LidarVisualization : MonoBehaviour
{
    public DroneController droneController;
    public float visualizationDuration = 0.1f;
    public float markerSize = 0.5f;
    public Color obstacleColor = Color.green;
    public Color droneColor = Color.red;
    public Color defaultColor = Color.grey;
    public Color lineColor = Color.red;
    public Color terrainColor = Color.grey;
    private List<GameObject> visualizationObjects = new List<GameObject>();
    private Material obstacleMaterial;
    private Material droneMaterial;
    private Material defaultMaterial;
    private Material lineMaterial;

    private void Start()
    {
        SetupMaterials();
        SetupVisualizationEnvironment();
    }

    private void SetupMaterials()
    {
        obstacleMaterial = new Material(Shader.Find("Standard"));
        obstacleMaterial.color = obstacleColor;

        droneMaterial = new Material(Shader.Find("Standard"));
        droneMaterial.color = droneColor;

        defaultMaterial = new Material(Shader.Find("Standard"));
        defaultMaterial.color = defaultColor;

        lineMaterial = new Material(Shader.Find("Standard"));
        lineMaterial.color = lineColor;
    }

    private void SetupVisualizationEnvironment()
    {
        // Apply green color to objects tagged as 'datasenditem'
        GameObject[] datasenditems = GameObject.FindGameObjectsWithTag("datasenditem");
        foreach (GameObject item in datasenditems)
        {
            ApplyMaterialToObject(item, obstacleMaterial);
        }

        // Apply grey color to objects tagged as 'REST'
        GameObject[] restObjects = GameObject.FindGameObjectsWithTag("REST");
        foreach (GameObject item in restObjects)
        {
            if (item.GetComponent<Terrain>() != null)
            {
                ApplyColorToTerrain(item.GetComponent<Terrain>(), terrainColor);
            }
            else
            {
                ApplyMaterialToObject(item, defaultMaterial);
            }
        }

        // Apply red color to the drone (assuming it has the "Player" tag)
        GameObject drone = GameObject.FindGameObjectWithTag("Player");
        if (drone != null)
        {
            ApplyMaterialToObject(drone, droneMaterial);
        }
    }
    private void ApplyColorToTerrain(Terrain terrain, Color color)
    {
        // Disable grass and trees
        terrain.drawTreesAndFoliage = false;

        // Create a new terrain layer with the desired color
        TerrainLayer greyLayer = new TerrainLayer();
        greyLayer.diffuseTexture = CreateColorTexture(color);
        greyLayer.tileSize = new Vector2(1, 1);

        // Apply the new layer to the entire terrain
        terrain.terrainData.terrainLayers = new TerrainLayer[] { greyLayer };

        // Set the heightmap alphamap to use only this layer
        float[,,] alphamaps = new float[terrain.terrainData.alphamapWidth, terrain.terrainData.alphamapHeight, 1];
        for (int y = 0; y < terrain.terrainData.alphamapHeight; y++)
        {
            for (int x = 0; x < terrain.terrainData.alphamapWidth; x++)
            {
                alphamaps[x, y, 0] = 1;
            }
        }
        terrain.terrainData.SetAlphamaps(0, 0, alphamaps);

        // Clear all trees and details
        terrain.terrainData.treeInstances = new TreeInstance[0];
        for (int i = 0; i < terrain.terrainData.detailPrototypes.Length; i++)
        {
            int[,] details = new int[terrain.terrainData.detailWidth, terrain.terrainData.detailHeight];
            terrain.terrainData.SetDetailLayer(0, 0, i, details);
        }
    }

    private Texture2D CreateColorTexture(Color color)
    {
        Texture2D texture = new Texture2D(1, 1);
        texture.SetPixel(0, 0, color);
        texture.Apply();
        return texture;
    }
    private void ApplyMaterialToObject(GameObject obj, Material material)
    {
        Renderer renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            renderer.material = material;
        }

        // Apply material to child objects if they exist
        foreach (Transform child in obj.transform)
        {
            ApplyMaterialToObject(child.gameObject, material);
        }
    }

    private void Update()
    {
        droneController.PerformLidarScan();
        ClearVisualization();
        VisualizeDrone();
        VisualizePaths();
    }

    private void ClearVisualization()
    {
        foreach (var obj in visualizationObjects)
        {
            Destroy(obj);
        }
        visualizationObjects.Clear();
    }

    private void VisualizeDrone()
    {
        GameObject droneVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        droneVisual.transform.position = droneController.transform.position;
        droneVisual.transform.rotation = droneController.transform.rotation;
        droneVisual.transform.localScale = Vector3.one * markerSize;
        Renderer renderer = droneVisual.GetComponent<Renderer>();
        renderer.material = droneMaterial;
        visualizationObjects.Add(droneVisual);
    }

    private void VisualizePaths()
    {
        GameObject baseStation = GameObject.FindGameObjectWithTag("basestation");
        if (baseStation == null)
        {
            Debug.LogError("Base station not found. Make sure it has the 'basestation' tag.");
            return;
        }

        Vector3 previousPosition = baseStation.transform.position;
        foreach (Transform waypoint in droneController.waypoints)
        {
            DrawLine(previousPosition, waypoint.position, lineColor);
            previousPosition = waypoint.position;
        }
    }

    private void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        GameObject line = new GameObject("Line");
        LineRenderer lineRenderer = line.AddComponent<LineRenderer>();
        lineRenderer.material = lineMaterial;
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);
        visualizationObjects.Add(line);
    }
    
}

*/
using UnityEngine;
using System.Collections.Generic;

public class LidarVisualization : MonoBehaviour
{
    public DroneController droneController;
    public Color lineColor = Color.red;
    private Material lineMaterial;
    private List<GameObject> visualizationObjects = new List<GameObject>();

    private void Start()
    {
        SetupMaterials();
    }

    private void SetupMaterials()
    {
        lineMaterial = new Material(Shader.Find("Standard"));
        lineMaterial.color = lineColor;
    }

    private void Update()
    {
        ClearVisualization();
        VisualizePaths();
    }

    private void ClearVisualization()
    {
        foreach (var obj in visualizationObjects)
        {
            Destroy(obj);
        }
        visualizationObjects.Clear();
    }

    private void VisualizePaths()
    {
        GameObject baseStation = GameObject.FindGameObjectWithTag("basestation");
        if (baseStation == null)
        {
            Debug.LogError("Base station not found. Make sure it has the 'basestation' tag.");
            return;
        }

        Vector3 previousPosition = baseStation.transform.position;
        foreach (Transform waypoint in droneController.waypoints)  // Ensure 'waypoints' is defined in DroneController
        {
            DrawLine(previousPosition, waypoint.position, lineColor);
            previousPosition = waypoint.position;
        }
    }

    private void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        GameObject line = new GameObject("Line");
        LineRenderer lineRenderer = line.AddComponent<LineRenderer>();
        lineRenderer.material = lineMaterial;
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.startWidth = 0.1f;
        lineRenderer.endWidth = 0.1f;
        lineRenderer.positionCount = 2;
        lineRenderer.SetPosition(0, start);
        lineRenderer.SetPosition(1, end);
        visualizationObjects.Add(line);
    }
}
