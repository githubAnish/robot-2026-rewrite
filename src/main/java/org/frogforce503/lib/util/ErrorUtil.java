package org.frogforce503.lib.util;

public class ErrorUtil {
    private ErrorUtil() {}
    
    public static String attachJavaClassName(Class<?> className) {
        return " --- Returned from " + className.getSimpleName() + ".java";
    }
}