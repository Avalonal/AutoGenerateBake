using LinkBake;
using UnityEngine;

public class LinkBakeDrawHelper : MonoBehaviour
{
    private BakeLink _window;

    public void Init(BakeLink window)
    {
        _window = window;
    }

    void OnDrawGizmos()
    {
        _window.OnDrawGizmos();
    }
}
