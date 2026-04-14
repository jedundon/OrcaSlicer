# Known Issues & Review Notes — Hole Fill Color Tool

## Fixed in second commit (29997c4f)
1. **m_rr.mesh_id mapping bug** — was comparing against raw volume index instead of model-part-only index. Would cause missed hits or wrong volume selection on objects with modifier volumes.
2. **Hardcoded scaling factor** — used `1e6` instead of `scale_()` macro. Would produce wrong geometry on large-format printers where SCALING_FACTOR differs.
3. **Called private method** — `update_raycast_cache()` is private in `GLGizmoPainterBase`. Removed the call; relying on base class Moving events instead.

## Remaining concerns (need build to verify)

### High Priority
4. **Side wall winding order** — The side wall triangles `(f0, b0, f1)` and `(f1, b0, b1)` may have incorrect winding depending on the boundary loop orientation. If the plug renders inside-out, swap the triangle vertex order. Test: load a model with engraved text, fill a hole, slice, and check the preview for inside-out faces.

5. **Tool button rendering** — Using PUA codepoint `0xF0FF` as the tool icon. This glyph doesn't exist in the icon font. Possible outcomes:
   - Renders as empty/blank button (acceptable for MVP)
   - Crashes the ImGui font renderer (unlikely but possible)
   - Renders as a tofu box □ (fine)
   
   **Mitigation**: If it crashes, replace `HoleFillToolIcon` with one of the existing icon constants temporarily.

6. **Coordinate space for plug placement** — The plug mesh is generated in the volume's local coordinate space and `set_transformation()` copies the source volume's transform. This should be correct, but if the plug appears offset, the issue is likely that `m_rr.hit` is in a different coordinate space than expected. Test with a non-origin-centered model to verify.

### Medium Priority
7. **ExPolygon orientation** — The `is_clockwise()` check and `reverse()` in PlugGenerator assumes the Slic3r Polygon orientation convention. If the tessellation produces degenerate triangles, this might need investigation.

8. **Gizmo re-initialization** — After adding a new ModelVolume, the gizmo's `m_triangle_selectors` may be stale (they're built from the original volume list). The user might need to exit and re-enter the gizmo. Consider calling `init_model_triangle_selectors()` after the fill.

9. **Undo/redo interaction** — The `Plater::TakeSnapshot` should work, but undoing a hole fill while still in the gizmo might leave inconsistent state. Test: fill a hole, press Ctrl+Z, verify the plug disappears cleanly.

### Low Priority
10. **Multiple holes per face** — `find_nearest_hole` returns only the closest hole to the click point. Filling multiple holes requires multiple clicks. This is by design but could be enhanced with "fill all holes" later.

11. **Thin walls near hole boundaries** — If the plug depth exceeds the model wall thickness at the hole location, the plug will protrude through the back. No runtime check for this yet.

12. **HoleFinder performance** — The flood-fill uses `its_face_neighbors()` which builds a full adjacency structure. For very high-poly meshes (>1M triangles) this could cause a noticeable pause on click. Not a problem for typical models.
