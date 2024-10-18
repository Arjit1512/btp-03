using UnityEngine;

public class CubeColorChange : MonoBehaviour
{
    public Color newColor = Color.red;

    void Start()
    {
        Renderer renderer = GetComponent<Renderer>();
        Material material = renderer.material;
        material.color = newColor;
    }
}
