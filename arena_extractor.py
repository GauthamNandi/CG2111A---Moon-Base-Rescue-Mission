import numpy as np

def extract_arena_rectangles(map_array, map_size_pixels, map_size_meters):
    try:
        import cv2
    except ImportError:
        print("[arena_extractor] ERROR: opencv-python is not installed.")
        return []

    binary_mask = np.uint8((map_array < 120) * 255)
    pixel_size_m = map_size_meters / float(map_size_pixels)
    
    rectangles_m = []
    inner_mask = binary_mask.copy()
    
    # Boundary principle axes defaults
    u1 = np.array([1.0, 0.0])
    u2 = np.array([0.0, 1.0])
    C_b = np.array([map_size_pixels / 2.0, map_size_pixels / 2.0])
    u1_half = map_size_pixels
    u2_half = map_size_pixels
    
    # 1. IMPROVED BOUNDARY EXTRACTION
    kernel_large = cv2.getStructuringElement(cv2.MORPH_RECT, (45, 45))
    closed_for_bound = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel_large)
    
    contours_b, _ = cv2.findContours(closed_for_bound, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours_b:
        main_boundary_cnt = max(contours_b, key=cv2.contourArea)
        rect_b = cv2.minAreaRect(main_boundary_cnt)
        
        # Clip boundary to be between 2.8 and 3.3 meters
        (cx_b, cy_b), (w_px_b, h_px_b), angle_b = rect_b
        w_m_b = np.clip(w_px_b * pixel_size_m, 2.8, 3.3)
        h_m_b = np.clip(h_px_b * pixel_size_m, 2.8, 3.3)
        
        w_px_b = w_m_b / pixel_size_m
        h_px_b = h_m_b / pixel_size_m
        rect_b_clipped = ((cx_b, cy_b), (w_px_b, h_px_b), angle_b)
        
        box_b = cv2.boxPoints(rect_b_clipped)
        
        # Calculate principle axes of the clipped boundary
        v1 = box_b[1] - box_b[0]
        v1_len = np.linalg.norm(v1)
        if v1_len > 1e-3:
            u1 = v1 / v1_len
            u2 = np.array([-u1[1], u1[0]]) # Orthogonal axis

        C_b = np.array([cx_b, cy_b])

        # Define the strict inside zone using the CLIPPED sizes.
        # Subtract a 10cm margin so inner blocks never touch the boundary wall.
        margin_px = 0.10 / pixel_size_m
        u1_half = (w_px_b / 2.0) - margin_px
        u2_half = (h_px_b / 2.0) - margin_px
            
        box_b_m = [(x * pixel_size_m, y * pixel_size_m) for x, y in box_b]
        rectangles_m.append(box_b_m)
        
        # Scrub out the boundary from the inner walls mask
        cv2.drawContours(inner_mask, [main_boundary_cnt], -1, 0, thickness=55)
        
    # 2. ISOLATE INNER BLOCKS
    kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    inner_mask = cv2.morphologyEx(inner_mask, cv2.MORPH_CLOSE, kernel_small)
    inner_mask = cv2.morphologyEx(inner_mask, cv2.MORPH_OPEN, kernel_small)
    
    contours_i, _ = cv2.findContours(inner_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    inner_blocks = []
    for cnt in contours_i:
        area_px = cv2.contourArea(cnt)
        if area_px >= 5:
            inner_blocks.append((area_px, cnt))
            
    # Sort by confidence
    inner_blocks.sort(key=lambda x: x[0], reverse=True)
    
    # Overlap tracking mask
    overlap_mask = np.zeros_like(binary_mask)
    
    L_px = 0.30 / pixel_size_m
    W_px = 0.05 / pixel_size_m
    
    max_blocks = 10
    blocks_added = 0
    
    # 3. DRAW AT MOST 10 INNER BLOCKS
    for area_px, cnt in inner_blocks:
        if blocks_added >= max_blocks:
            break
            
        rect = cv2.minAreaRect(cnt)
        cx, cy = rect[0]
        C = np.array([cx, cy])
        
        # Find the orientation of the current blob
        box_i = cv2.boxPoints(rect)
        d1 = np.linalg.norm(box_i[1] - box_i[0])
        d2 = np.linalg.norm(box_i[2] - box_i[1])
        
        if d1 > d2:
            v_i = box_i[1] - box_i[0]
            v_len = d1
        else:
            v_i = box_i[2] - box_i[1]
            v_len = d2
            
        if v_len < 1e-3:
            continue
        u_i = v_i / v_len
        
        # Snap to boundary axes
        if abs(np.dot(u_i, u1)) > abs(np.dot(u_i, u2)):
            dir_L = u1
            dir_W = u2
        else:
            dir_L = u2
            dir_W = u1
            
        corners = np.array([
            C + (L_px/2)*dir_L + (W_px/2)*dir_W,
            C - (L_px/2)*dir_L + (W_px/2)*dir_W,
            C - (L_px/2)*dir_L - (W_px/2)*dir_W,
            C + (L_px/2)*dir_L - (W_px/2)*dir_W
        ], dtype=np.float32)
        
        # --- NEW: Check if strictly inside the arena bounds ---
        # Projects every perfectly generated corner to the bounds of the arena vector space.
        inside_boundary = True
        for corner in corners:
            V = corner - C_b
            x_proj = abs(np.dot(V, u1))
            y_proj = abs(np.dot(V, u2))
            
            if x_proj > u1_half or y_proj > u2_half:
                inside_boundary = False
                break
                
        if not inside_boundary:
            continue
            
        # Check for block-to-block overlap
        temp_mask = np.zeros_like(binary_mask)
        cv2.fillPoly(temp_mask, [np.int32(corners)], 255)
        
        if cv2.bitwise_and(overlap_mask, temp_mask).any():
            continue  
            
        cv2.bitwise_or(overlap_mask, temp_mask, dst=overlap_mask)
        blocks_added += 1
        
        box_m = [(x * pixel_size_m, y * pixel_size_m) for x, y in corners]
        rectangles_m.append(box_m)
        
    return rectangles_m
